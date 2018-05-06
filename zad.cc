#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/energy-module.h"
#include "ns3/csma-module.h"
#include "ns3/gnuplot.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiSimpleAdhocGrid");

#define GRID_DISTANCE 200
#define PACKET_SIZE 200

Gnuplot2dDataset data[25];
/// Trace function for remaining energy at node.

void RemainingEnergy(uint32_t i, double oldValue, double remainingEnergy) {
    double tm = Simulator::Now().GetSeconds();
    data[i].Add(tm, remainingEnergy);
}

Vector randomManPosition(int x, int y, Vector m_position) {
    int r, s, edge;
    edge = rand() % 4;
    r = rand() % x;
    s = rand() % y;
    switch (edge) {
        case 0:
            //(0,s)
            m_position.x = 0;
            m_position.y = s;
            break;
        case 1:
            //(r,0)
            m_position.x = r;
            m_position.y = 0;
            break;
        case 2:
            //(x,s)
            m_position.x = x;
            m_position.y = s;
            break;
        case 3:
            //(r,y)
            m_position.x = r;
            m_position.y = y;
            break;
    }
    return m_position;
}

void SensorRxCallback(Ptr<Socket> sendSocket, Ptr<Socket> socket) {
    //    NS_LOG_UNCOND("Sensor received -> Sending");
    while (socket->Recv()) {
        sendSocket->Send(Create<Packet>(PACKET_SIZE));
    }
}

//void TestFarmer(Ptr<Socket> socket) {
//    NS_LOG_UNCOND("SENDING TO FARMER");
//    socket->Send(Create<Packet> (PACKET_SIZE));
//}

void FarmerRxCallback(Ptr<Socket> socket) {
    while (socket->Recv()) {
        //        NS_LOG_UNCOND("FARMER RECEIVED");
        std::cout << ".";
    }
}

static void farmerSend(Ptr<Socket> socket[]) {
    for (int i = 0; i < 25; i++) {
        //        NS_LOG_UNCOND("FARMER SEND");
        socket[i]->Send(Create<Packet>(PACKET_SIZE));
    }
    std::cout << "\nBC\n";
}

static void ManWalking(Ptr<ConstantPositionMobilityModel> cvMob, Ptr<Socket> socket[]) {
    Vector m_position = cvMob->GetPosition();
    cvMob->SetPosition(randomManPosition(4 * GRID_DISTANCE, 4 * GRID_DISTANCE, m_position));
    //    NS_LOG_UNCOND("FARMER MOVED");
    Simulator::Schedule(Seconds(30.0), &farmerSend, socket);
    Simulator::Schedule(Seconds(35.0), &ManWalking, cvMob, socket);
}

int main(int argc, char *argv[]) {

    Gnuplot graf("battery.svg");
    graf.SetTerminal("svg");
    graf.SetTitle("Baterie");
    graf.SetLegend("Cas[s]", "Ostavajuca Energia v Baterii[J]");

    for (int i = 0; i < 25; i++) {
        data[i].SetTitle("node " + std::to_string(i));
        data[i].SetStyle(Gnuplot2dDataset::LINES);
    }


    uint32_t numNodes = 25; // by default, 5x5

    std::string phyMode("DsssRate1Mbps");
    Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue("2200"));
    Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("20"));
    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
            StringValue(phyMode));

    NodeContainer c;
    c.Create(numNodes);

    NodeContainer f;
    f.Create(1);

    // The below set of helpers will help us to put together the wifi NICs we want
    WifiHelper wifi;

    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    // set it to zero; otherwise, gain will be added
    wifiPhy.Set("RxGain", DoubleValue(-10));
    // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
    wifiPhy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11_RADIO);

    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    wifiPhy.SetChannel(wifiChannel.Create());

    // Add an upper mac and disable rate control
    WifiMacHelper wifiMac;
    wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
            "DataMode", StringValue(phyMode),
            "ControlMode", StringValue(phyMode));
    // Set it to adhoc mode
    wifiMac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, c);
    NetDeviceContainer fdevices = wifi.Install(wifiPhy, wifiMac, f);

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
            "MinX", DoubleValue(0.0),
            "MinY", DoubleValue(0.0),
            "DeltaX", DoubleValue(GRID_DISTANCE),
            "DeltaY", DoubleValue(GRID_DISTANCE),
            "GridWidth", UintegerValue(5),
            "LayoutType", StringValue("RowFirst"));

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    mobility.Install(c);

    MobilityHelper fmobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0, 0, 0));
    fmobility.SetPositionAllocator(positionAlloc);

    fmobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    fmobility.Install(f);

    Ptr<ConstantPositionMobilityModel> cvMob = f.Get(0)->GetObject<ConstantPositionMobilityModel>();

    /** Energy Model **/
    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(250));
    EnergySourceContainer sources = basicSourceHelper.Install(c);
    WifiRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.0174));
    DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install(devices, sources);

    // Enable OLSR
    OlsrHelper olsr;
    Ipv4StaticRoutingHelper staticRouting;

    Ipv4ListRoutingHelper list;
    list.Add(staticRouting, 0);
    list.Add(olsr, 10);

    InternetStackHelper internet;
    internet.SetRoutingHelper(list); // has effect on the next Install ()
    internet.Install(c);
    internet.Install(f);

    NS_LOG_INFO("Build Topology.");
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", DataRateValue(DataRate(5000000)));
    csma.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));

    csma.Install(c);
    csma.Install(f);

    Ipv4AddressHelper ipv4;
    NS_LOG_INFO("Assign IP Addresses.");
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer IpContainter = ipv4.Assign(devices);
    Ipv4InterfaceContainer fIpContainter = ipv4.Assign(fdevices);

    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

    Ptr<Socket> farmerRx = Socket::CreateSocket(f.Get(0), tid);
    InetSocketAddress farmerRxIsoc = InetSocketAddress(Ipv4Address::GetAny(), 80);
    farmerRx->SetAllowBroadcast(true);
    farmerRx->Bind(farmerRxIsoc);
    farmerRx->SetRecvCallback(MakeCallback(&FarmerRxCallback));

    Ptr<Socket> farmerBcArr[25];

    for (uint32_t i = 0; i < c.GetN(); i++) {
        Ptr<Socket> sensorRxSocket = Socket::CreateSocket(c.Get(i), tid);
        InetSocketAddress sensorRxIsoc = InetSocketAddress(Ipv4Address::GetAny(), 80);
        sensorRxSocket->SetAllowBroadcast(true);
        sensorRxSocket->Bind(sensorRxIsoc);

        Ptr<Socket> sensorTxSocket = Socket::CreateSocket(c.Get(i), tid);
        InetSocketAddress sensorTxIsoc = InetSocketAddress(fIpContainter.GetAddress(0), 80);
        sensorTxSocket->SetAllowBroadcast(true);
        sensorTxSocket->Connect(sensorTxIsoc);

        sensorRxSocket->SetRecvCallback(MakeBoundCallback(&SensorRxCallback, sensorTxSocket));

        farmerBcArr[i] = Socket::CreateSocket(f.Get(0), tid);
        InetSocketAddress farmerBcIsoc = InetSocketAddress(IpContainter.GetAddress(i), 80);
        farmerBcArr[i]->Connect(farmerBcIsoc);
    }
    Simulator::Schedule(Seconds(30.0), &ManWalking, cvMob, farmerBcArr);

    for (int i = 0; i < 25; i++) {
        Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource>(sources.Get(i));
        basicSourcePtr->TraceConnectWithoutContext("RemainingEnergy", MakeBoundCallback(&RemainingEnergy, i));
    }

    // Output what we are doing
    NS_LOG_UNCOND("Testing from node "
            << " with grid distance " << GRID_DISTANCE);

    Simulator::Stop(Seconds(300.0));

    AnimationInterface anim("wireless-animation.xml"); // Mandatory
    for (uint32_t i = 0; i < c.GetN(); ++i) {
        std::string str = "STA";
        str += '1' + i;

        anim.UpdateNodeColor(c.Get(i), 255, 0, 0); // Optional
        anim.UpdateNodeSize(c.Get(i)->GetId(), 3.0, 3.0);
    }
    anim.UpdateNodeDescription(f.Get(0), "FARMER");
    anim.UpdateNodeSize(f.Get(0)->GetId(), 15.0, 15.0);
    anim.UpdateNodeColor(f.Get(0), 255, 0, 0);
    anim.EnablePacketMetadata(); // Optional
    anim.EnableIpv4RouteTracking("routingtable-wireless.xml", Seconds(0), Seconds(5), Seconds(0.25)); //Optional
    anim.EnableWifiMacCounters(Seconds(0), Seconds(10)); //Optional
    anim.EnableWifiPhyCounters(Seconds(0), Seconds(10)); //Optional

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
    //    int j = 0;
    int lostPackets = 0;
    int rxPackets = 0;
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i) {
        lostPackets += i->second.lostPackets;
        rxPackets += i->second.rxPackets;
    }

    double avgLoss = ((double) lostPackets / (double) (lostPackets + rxPackets)) * 100;

    std::cout << "\n"
            << PACKET_SIZE << "B - " << GRID_DISTANCE << "M - " << avgLoss << "% loss\n";

    Simulator::Destroy();

    for (int i = 0; i < 25; i++) {
        graf.AddDataset(data[i]);
    }
    std::ofstream plotFile("graf.plt");
    graf.GenerateOutput(plotFile);
    plotFile.close();
    if (system("gnuplot graf.plt"))
        ;

    return 0;
}