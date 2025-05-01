#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/zigbee-module.h"
#include "ns3/simulator.h"
#include "ns3/netanim-module.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/spectrum-channel.h"
#include "ns3/single-model-spectrum-channel.h"

#include <fstream>

using namespace ns3;
using namespace ns3::lrwpan;
using namespace ns3::zigbee;

ZigbeeStackContainer zigbeeStacks;

// Metrics tracking
std::map<Mac16Address, Time> joinTimes;
uint32_t packetsSent = 0, packetsReceived = 0;

// File streams for metrics
std::ofstream delayFile("delay.txt", std::ios::out);
std::ofstream throughputFile("throughput.txt", std::ios::out);
std::ofstream pdrFile("pdr.txt", std::ios::out);

// Simulated Application: Sensor sending data to coordinator
void SendSensorData(Ptr<ZigbeeStack> sender, Ptr<ZigbeeStack> coordinator, uint32_t size)
{
    Ptr<Packet> packet = Create<Packet>(size);
    NldeDataRequestParams params;
    params.m_dstAddrMode = ns3::zigbee::UCST_BCST;
    params.m_dstAddr = coordinator->GetNwk()->GetNetworkAddress();
    params.m_nsduHandle = 1;
    params.m_discoverRoute = ENABLE_ROUTE_DISCOVERY;
    Simulator::ScheduleNow(&ZigbeeNwk::NldeDataRequest, sender->GetNwk(), params, packet);
    packetsSent++;
    NS_LOG_UNCOND("Packet sent from node " << sender->GetNode()->GetId());
    Simulator::Schedule(Seconds(10), &SendSensorData, sender, coordinator, size);
}

static void NwkNetworkFormationConfirm(Ptr<ZigbeeStack> stack, NlmeNetworkFormationConfirmParams params)
{
    std::cout << "NlmeNetworkFormationConfirmStatus = " << params.m_status << "\n";
}

static void NwkNetworkDiscoveryConfirm(Ptr<ZigbeeStack> stack, NlmeNetworkDiscoveryConfirmParams params)
{
    if (params.m_status == NwkStatus::SUCCESS)
    {
        std::cout << "    Network discovery confirm Received. Networks found "
                  << "(" << params.m_netDescList.size() << ")\n";

        for (const auto &netDescriptor : params.m_netDescList)
        {
            std::cout << "      ExtPanID: 0x" << std::hex << netDescriptor.m_extPanId << std::dec
                      << "\n"
                      << "      CH:  " << static_cast<uint32_t>(netDescriptor.m_logCh) << std::hex
                      << "\n"
                      << "      Pan Id: 0x" << netDescriptor.m_panId << std::hex << "\n"
                      << "      stackprofile: " << std::dec
                      << static_cast<uint32_t>(netDescriptor.m_stackProfile) << "\n"
                      << "      ----------------\n ";
        }

        NlmeJoinRequestParams joinParams;

        zigbee::CapabilityInformation capaInfo;
        capaInfo.SetDeviceType(ROUTER);
        capaInfo.SetAllocateAddrOn(true);

        joinParams.m_rejoinNetwork = zigbee::JoiningMethod::ASSOCIATION;
        joinParams.m_capabilityInfo = capaInfo.GetCapability();
        joinParams.m_extendedPanId = params.m_netDescList[0].m_extPanId;

        Simulator::ScheduleNow(&ZigbeeNwk::NlmeJoinRequest, stack->GetNwk(), joinParams);
    }
    else
    {
        std::cout << " WARNING: Unable to discover networks | status: " << params.m_status << "\n";
    }
}

static void NwkJoinConfirm(Ptr<ZigbeeStack> stack, NlmeJoinConfirmParams params)
{
    if (params.m_status == NwkStatus::SUCCESS)
    {
        Mac16Address shortAddr = params.m_networkAddress;
        joinTimes[shortAddr] = Simulator::Now(); // Record the time the node joined the network
        NlmeStartRouterRequestParams startRouterParams;
        Simulator::ScheduleNow(&ZigbeeNwk::NlmeStartRouterRequest,
                               stack->GetNwk(), startRouterParams);
    }
}

static void NwkDataIndication(Ptr<ZigbeeStack> stack, NldeDataIndicationParams params, Ptr<Packet> p)
{
    packetsReceived++;
    Time delay = Simulator::Now() - joinTimes[stack->GetNwk()->GetNetworkAddress()]; // Measure end-to-end delay
    NS_LOG_UNCOND("End-to-end delay for node " << stack->GetNode()->GetId() << ": " << delay.GetSeconds() << "s");
    delayFile << delay.GetSeconds() << "\n";
    NS_LOG_UNCOND("Received packet at node " << stack->GetNode()->GetId()
                                             << " of size " << p->GetSize()
                                             << " at " << Simulator::Now().GetSeconds() << "s");
}

static void NwkRouteDiscoveryConfirm(Ptr<ZigbeeStack> stack, NlmeRouteDiscoveryConfirmParams params)
{
    std::cout << "NlmeRouteDiscoveryConfirmStatus = " << params.m_status << "\n";
}

// Light: Receives a random 16-byte command
void ReceiveLightCommand(Ptr<ZigbeeStack> light)
{
    Ptr<Packet> packet = Create<Packet>(16);
    // Logic to process the command
    Simulator::Schedule(Seconds(60), &ReceiveLightCommand, light); // On average, 1 command per minute
}

// Lock: Sends a 24-byte status notification when there is a change
void SendLockStatus(Ptr<ZigbeeStack> lock)
{
    Ptr<Packet> packet = Create<Packet>(24);
    // Logic to process the status notification
    Simulator::Schedule(Seconds(rand() % 60), &SendLockStatus, lock); // Randomly every minute
}

// Calculate overall network throughput
void CalculateThroughput()
{
    static uint32_t totalBytes = 0;
    totalBytes += packetsSent * 32 + packetsReceived * 32; // Update total bytes
    double throughput = totalBytes / Simulator::Now().GetSeconds();
    NS_LOG_UNCOND("Network Throughput: " << throughput << " bytes/s");
    throughputFile << throughput << "\n";
    Simulator::Schedule(Seconds(1), &CalculateThroughput);
}

int main(int argc, char *argv[])
{
    NodeContainer coordinator, sensors, lights, locks;
    coordinator.Create(1);
    sensors.Create(6); // 6 sensors
    lights.Create(4);  // 4 lights
    locks.Create(3);   // 3 locks

    NodeContainer all;
    all.Add(coordinator);
    all.Add(sensors);
    all.Add(lights);
    all.Add(locks);


    // Set up mobility
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0.0), "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(30.0), "DeltaY", DoubleValue(30.0),
                                  "GridWidth", UintegerValue(7), "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(all);

    // Set up channel, lrwpan, and ZigbeeStacks for nodes
    Ptr<SpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> propModel =
        CreateObject<LogDistancePropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    channel->AddPropagationLossModel(propModel);
    channel->SetPropagationDelayModel(delayModel);

    LrWpanHelper lrWpanHelper;
    lrWpanHelper.SetChannel(channel);
    NetDeviceContainer devices = lrWpanHelper.Install(all);
    lrWpanHelper.SetExtendedAddresses(devices);

    ZigbeeHelper zigbeeHelper;
    zigbeeStacks = zigbeeHelper.Install(devices);

    // NWK callbacks hooks
    for (auto i = zigbeeStacks.Begin(); i != zigbeeStacks.End(); i++)
    {
        Ptr<ZigbeeStack> z = *i;
        z->GetNwk()->SetNlmeNetworkFormationConfirmCallback(MakeBoundCallback(&NwkNetworkFormationConfirm, z));
        z->GetNwk()->SetNlmeRouteDiscoveryConfirmCallback(MakeBoundCallback(&NwkRouteDiscoveryConfirm, z));
        z->GetNwk()->SetNldeDataIndicationCallback(MakeBoundCallback(&NwkDataIndication, z));
        z->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(MakeBoundCallback(&NwkNetworkDiscoveryConfirm, z));
        z->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, z));
    }

    // Set up Zigbee Coordinator and start the network
    Ptr<ZigbeeStack> coordStack = zigbeeStacks.Get(0);
    NlmeNetworkFormationRequestParams formParams;
    formParams.m_scanChannelList.channelPageCount = 1;
    formParams.m_scanChannelList.channelsField[0] = ALL_CHANNELS;
    formParams.m_scanDuration = 0;
    formParams.m_superFrameOrder = 15;
    formParams.m_beaconOrder = 15;
    Simulator::ScheduleNow(&ZigbeeNwk::NlmeNetworkFormationRequest,
                           coordStack->GetNwk(), formParams);

    // Schedule other nodes to discover and join
    for (uint32_t i = 1; i < zigbeeStacks.GetN(); i++)
    {
        Ptr<ZigbeeStack> stack = zigbeeStacks.Get(i);
        NlmeNetworkDiscoveryRequestParams discParams;
        discParams.m_scanChannelList.channelPageCount = 1;
        discParams.m_scanChannelList.channelsField[0] = 0x00007800;
        discParams.m_scanDuration = 2;
        Simulator::Schedule(Seconds(2 + i * 2), &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                            stack->GetNwk(), discParams);
    }

    // Start sensor data traffic
    for (uint32_t i = 1; i <= sensors.GetN(); ++i)
    {
        Ptr<ZigbeeStack> s = zigbeeStacks.Get(i);
        Simulator::Schedule(Seconds(30 + i * 2), &SendSensorData, s, coordStack, 32);
    }

    // Start traffic simulation for lights and locks
    for (uint32_t i = 0; i < lights.GetN(); ++i)
    {
        Ptr<ZigbeeStack> l = zigbeeStacks.Get(i + sensors.GetN() + 1);
        Simulator::Schedule(Seconds(35), &ReceiveLightCommand, l);
    }
    for (uint32_t i = 0; i < locks.GetN(); ++i)
    {
        Ptr<ZigbeeStack> k = zigbeeStacks.Get(i + sensors.GetN() + lights.GetN() + 1);
        Simulator::Schedule(Seconds(35), &SendLockStatus, k);
    }

    // Collect network throughput
    Simulator::Schedule(Seconds(1), &CalculateThroughput);

    // NetAnim Visualization
    AnimationInterface anim("zigbee-iot.xml");
    anim.SetConstantPosition(coordinator.Get(0), 0, 0);
    for (uint32_t i = 0; i < sensors.GetN(); i++)
        anim.UpdateNodeColor(sensors.Get(i), 0, 255, 0); // Green: Sensors
    for (uint32_t i = 0; i < lights.GetN(); i++)
        anim.UpdateNodeColor(lights.Get(i), 0, 0, 255); // Blue: Lights
    for (uint32_t i = 0; i < locks.GetN(); i++)
        anim.UpdateNodeColor(locks.Get(i), 255, 255, 0); // Yellow: Locks

    anim.EnablePacketMetadata(); // Display packet data on NetAnim if needed
    anim.SetMaxPktsPerTraceFile(500000);

    Simulator::Stop(Seconds(1200));
    Simulator::Run();

    // Summary metrics
    std::cout << "\n========= Simulation Summary =========\n";
    std::cout << "Join Times:\n";
    for (const auto &[addr, time] : joinTimes)
    {
        std::cout << "Node (" << addr << ") joined at " << time.GetSeconds() << "s\n";
    }

    std::cout << "\nPackets Sent     : " << packetsSent << "\n";
    std::cout << "Packets Received : " << packetsReceived << "\n";
    std::cout << "Packet Delivery Ratio (PDR): "
              << (packetsSent > 0 ? (100.0 * packetsReceived / packetsSent) : 0.0)
              << " %\n";

    pdrFile << (packetsSent > 0 ? (100.0 * packetsReceived / packetsSent) : 0.0) << "\n";

    Simulator::Destroy();

    delayFile.close();
    throughputFile.close();
    pdrFile.close();
    return 0;
}
