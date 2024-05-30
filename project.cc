#include "ns3/building-allocator.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/buildings-helper.h"
#include "ns3/callback.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/double.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/forwarder-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/log.h"
#include "ns3/lora-device-address.h"
#include "ns3/lora-frame-header.h"
#include "ns3/lora-helper.h"
#include "ns3/lora-net-device.h"
#include "ns3/lora-phy.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/mobility-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/node-container.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/pointer.h"
#include "ns3/position-allocator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/ns2-mobility-helper.h"
#include <algorithm>
#include <ctime>
#include <unordered_map>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("AlohaThroughput");

int numDevices = 50;
int numGateways = 1;
double simTimeSeconds = 50; 

auto numPacketsSent = std::vector<int>(6, 0);
auto numPacketsReceived = std::vector<int>(6, 0);
std::unordered_map<uint32_t, Time> packetSentTime;
std::vector<Time> delays;

void TransmissionCallback(Ptr<const Packet> packet, uint32_t senderNodeId)
{
    NS_LOG_FUNCTION(packet << senderNodeId);
    LoraTag tag;
    packet->PeekPacketTag(tag);
    numPacketsSent.at(tag.GetSpreadingFactor() - 7)++;
    packetSentTime[packet->GetUid()] = Simulator::Now();
}

void PacketReceptionCallback(Ptr<const Packet> packet, uint32_t receiverNodeId)
{
    NS_LOG_FUNCTION(packet << receiverNodeId);
    LoraTag tag;
    packet->PeekPacketTag(tag);
    numPacketsReceived.at(tag.GetSpreadingFactor() - 7)++;
    
    auto it = packetSentTime.find(packet->GetUid());
    if (it != packetSentTime.end())
    {
        Time sentTime = it->second;
        Time receivedTime = Simulator::Now();
        Time delay = receivedTime - sentTime;
        delays.push_back(delay);
        packetSentTime.erase(it);
    }
}

void VerifyPositions(NodeContainer nodes)
{
    for (auto j = nodes.Begin(); j != nodes.End(); ++j)
    {
        Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel>();
        Vector position = mobility->GetPosition();
        NS_LOG_INFO("Node " << (*j)->GetId() << " position: " << position);
    }
    Simulator::Schedule(Seconds(10.0), &VerifyPositions, nodes); // Her 10 saniyede bir kontrol et
}

int main(int argc, char* argv[])
{
    std::string interferenceMatrix = "aloha";
    std::string traceFile = "/home/bilmuh/ns-allinone-3.41/ns-3.41/src/lorawan/examples/vanetmobility.tcl";

    CommandLine cmd(__FILE__);
    cmd.AddValue("numDevices", "Number of end devices to include in the simulation", numDevices);
    cmd.AddValue("simTime", "Simulation Time (s)", simTimeSeconds);
    cmd.AddValue("interferenceMatrix", "Interference matrix to use [aloha, goursaud]", interferenceMatrix);
    cmd.AddValue("traceFile", "The path to the NS2 movement trace file", traceFile);
    cmd.Parse(argc, argv);

    int appPeriodSeconds = simTimeSeconds;

    //LogComponentEnable("AlohaThroughput", LOG_LEVEL_ALL);

    if (interferenceMatrix == "aloha")
    {
        LoraInterferenceHelper::collisionMatrix = LoraInterferenceHelper::ALOHA;
    }
    else if (interferenceMatrix == "goursaud")
    {
        LoraInterferenceHelper::collisionMatrix = LoraInterferenceHelper::GOURSAUD;
    }

    // Create a set of nodes
    NodeContainer endDevices;
    endDevices.Create(numDevices);

    // Mobility
    Ns2MobilityHelper ns2 = Ns2MobilityHelper(traceFile);
    ns2.Install();

    // Check initial positions of nodes
    VerifyPositions(endDevices);

    // Create the lora channel object
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76);
    loss->SetReference(1, 7.7);

    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

    // Create the LoraPhyHelper
    LoraPhyHelper phyHelper = LoraPhyHelper();
    phyHelper.SetChannel(channel);

    // Create the LorawanMacHelper
    LorawanMacHelper macHelper = LorawanMacHelper();
    macHelper.SetRegion(LorawanMacHelper::ALOHA);

    // Create the LoraHelper
    LoraHelper helper = LoraHelper();
    helper.EnablePacketTracking(); // Output filename

    // Create the NetworkServerHelper
    NetworkServerHelper nsHelper = NetworkServerHelper();

    // Create the ForwarderHelper
    ForwarderHelper forHelper = ForwarderHelper();

    // Create the LoraNetDevices of the end devices
    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen = CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);

    macHelper.SetAddressGenerator(addrGen);
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);
    helper.Install(phyHelper, macHelper, endDevices);

    // Connect trace sources
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> node = *j;
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<LoraPhy> phy = loraNetDevice->GetPhy();
    }

    // Create the gateway nodes (allocate them uniformly on the disc)
    NodeContainer gateways;
    gateways.Create(numGateways);

    // Gateway mobility using the positions from the ns-2 trace file
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
    allocator->Add(Vector(0.0, 0.0, 15.0));
    MobilityHelper mobility;
    mobility.SetPositionAllocator(allocator);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gateways);

    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);

    NS_LOG_DEBUG("Completed configuration");

    Time appStopTime = Seconds(simTimeSeconds);
    int pktSize = 50;
    PeriodicSenderHelper appHelper = PeriodicSenderHelper();
    appHelper.SetPeriod(Seconds(appPeriodSeconds));
    appHelper.SetPacketSize(pktSize);
    ApplicationContainer appContainer = appHelper.Install(endDevices);

    appContainer.Start(Seconds(0));
    appContainer.Stop(appStopTime);

    // Create the network server node
    Ptr<Node> netServer = CreateObject<Node>();

    // PointToPoint links between gateways and server
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));
    P2PGwRegistration_t gwRegistration;
    for (auto gw = gateways.Begin(); gw != gateways.End(); ++gw)
    {
        auto container = p2p.Install(netServer, *gw);
        auto serverP2PNetDev = DynamicCast<PointToPointNetDevice>(container.Get(0));
        gwRegistration.emplace_back(serverP2PNetDev, *gw);
    }

    nsHelper.SetGatewaysP2P(gwRegistration);
    nsHelper.SetEndDevices(endDevices);
    nsHelper.Install(netServer);

    forHelper.Install(gateways);

    // Install trace sources
    for (auto node = gateways.Begin(); node != gateways.End(); node++)
    {
        (*node)->GetDevice(0)->GetObject<LoraNetDevice>()->GetPhy()->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(PacketReceptionCallback));
    }

    for (auto node = endDevices.Begin(); node != endDevices.End(); node++)
    {
        (*node)->GetDevice(0)->GetObject<LoraNetDevice>()->GetPhy()->TraceConnectWithoutContext("StartSending", MakeCallback(TransmissionCallback));
    }

    LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);

    Simulator::Schedule(Seconds(10.0), &VerifyPositions, endDevices);

    Simulator::Stop(appStopTime + Hours(1));

    NS_LOG_INFO("Running simulation...");
    Simulator::Run();

    Simulator::Destroy();

    std::cout << "Packet Sent: " << numPacketsSent.at(0) << "\nPacket Received: " << numPacketsReceived.at(0) << "\nPacket Delivery Rate: " << (numPacketsSent.at(0) > 0 ? (static_cast<double>(numPacketsReceived.at(0)) / numPacketsSent.at(0)) * 100 : 0) << "%" << std::endl;

    if (!delays.empty())
    {
        Time totalDelay;
        for (const auto& d : delays)
        {
            totalDelay += d;
        }
        Time averageDelay = totalDelay / delays.size();
        std::cout << "Average delay: " << averageDelay.GetSeconds() << " seconds" << std::endl;
    }
    else
    {
        std::cout << "No packets received." << std::endl;
    }

    return 0;
}
