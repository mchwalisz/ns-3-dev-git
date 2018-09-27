/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 The Boeing Company
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors:
 *  Gary Pei <guangyu.pei@boeing.com>
 *  Tom Henderson <thomas.r.henderson@boeing.com>
 *  Luis Pacheco <luisbelem@gmail.com>
 *  Peishuo Li <pressthunder@gmail.com>
 */
#include "lr-wpan-tsch-helper.h"
#include <ns3/energy-module.h>
#include <ns3/lr-wpan-error-model.h>
#include <ns3/lr-wpan-tsch-net-device.h>
#include <ns3/mobility-model.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/friis-spectrum-propagation-loss.h>
#include <ns3/log.h>
#include "lr-wpan-radio-energy-model-helper.h"
#include "lr-wpan-energy-source-helper.h"

NS_LOG_COMPONENT_DEFINE ("LrWpanTschHelper");

namespace ns3 {

static void
AsciiLrWpanTschMacTransmitSinkWithContext (
  Ptr<OutputStreamWrapper> stream,
  std::string context,
  Ptr<const Packet> p)
{
  *stream->GetStream () << "t " << Simulator::Now ().GetSeconds () << " " << context << " " << *p << std::endl;
}

static void
AsciiLrWpanTschMacMaxRetriesSinkWithContext (
  Ptr<OutputStreamWrapper> stream,
  std::string context,
  Ptr<const Packet> p)
{
  *stream->GetStream () << "m " << Simulator::Now ().GetSeconds () << " " << context << " " << *p << std::endl;
}

static void
AsciiLrWpanTschMacMaxRetriesSinkWithoutContext (
  Ptr<OutputStreamWrapper> stream,
  Ptr<const Packet> p)
{
  *stream->GetStream () << "m " << Simulator::Now ().GetSeconds () << " " << *p << std::endl;
}

static void
AsciiLrWpanTschMacTransmitSinkWithoutContext (
  Ptr<OutputStreamWrapper> stream,
  Ptr<const Packet> p)
{
  *stream->GetStream () << "t " << Simulator::Now ().GetSeconds () << " " << *p << std::endl;
}

static void
EnergyTraceWithContext (Ptr<OutputStreamWrapper> stream, std::string context,uint32_t psize)
{
  if (psize > 0)
    {
      *stream->GetStream () << Simulator::Now ().GetSeconds () << " " << context << " Packet size: "  << psize << std::endl;
    }
  else
    {
      *stream->GetStream () << Simulator::Now ().GetSeconds () << " " << context << std::endl;
    }
}

static void
ConsumedEnergyTracing (Ptr<OutputStreamWrapper> stream_dev, std::string context_dev, std::string prePhyState, std::string curPhyState,
                       bool sourceEnergyUnlimited, double consumedEnergy,
                       double remainingEnergy, double totalConsumedEnergy)
{
  if(!sourceEnergyUnlimited)
     *stream_dev->GetStream () << Simulator::Now ().GetSeconds () << "s: "<< context_dev << " Device state switch to: " << curPhyState
                               << " from: " << prePhyState
                               << " Device consumed energy of previous state: " << consumedEnergy << "J"
                               << " Source remaining energy: " << remainingEnergy << "J"
                               <<" Total consumed energy: " << totalConsumedEnergy << "J" << std::endl;
  else
    *stream_dev->GetStream () << Simulator::Now ().GetSeconds () << "s: "<< context_dev << " Device state switch to: " << curPhyState
                              << " from: " << prePhyState
                              << " Device consumed energy of previous state: " << consumedEnergy << "J"
                              <<" Total consumed energy (Unlimited Source): " << totalConsumedEnergy << "J" << std::endl;
}

static void
ReceivedPowerTracing (Ptr<OutputStreamWrapper> stream_rec, uint32_t rxId, uint32_t txId, uint8_t channelNum, double receivedPower,
                      double fadingBiasPower)
{
   *stream_rec->GetStream () << Simulator::Now ().GetSeconds () << "s: " << "Received Node " << rxId
                            << " from Node " << txId << " on Channel: " << (int)channelNum <<" with power: " << receivedPower << " dBm"
                            <<" containing fading bias: " << fadingBiasPower << " dBm"
                            << " (with original " << receivedPower - fadingBiasPower << " dBm)"<< std::endl;
}

LrWpanTschHelper::LrWpanTschHelper (void)
{
  m_channel = CreateObject<SingleModelSpectrumChannel> ();
  Ptr<FriisSpectrumPropagationLossModel> model = CreateObject<FriisSpectrumPropagationLossModel> ();
  //Ptr<LogDistancePropagationLossModel> model = CreateObject<LogDistancePropagationLossModel> ();
  m_channel->AddSpectrumPropagationLossModel (model);
  m_slotframehandle = 0;
  m_numchannel = 16;
  m_numnode = 0;
  MinFadingBias = 0;
  MaxFadingBias = 0;
}

LrWpanTschHelper::LrWpanTschHelper (Ptr<SpectrumChannel> ch)
{
  m_channel = ch;
  m_slotframehandle = 0;
  m_numchannel = 16;
  m_numnode = 0;
  m_nakagami = false;
  m_isDay = true;
  MinFadingBias = 0;
  MaxFadingBias = 0;
}

LrWpanTschHelper::LrWpanTschHelper (Ptr<SpectrumChannel> ch, u_int32_t num_node, bool nakagami, bool isDay)
{
  m_channel = ch;
  m_slotframehandle = 0;
  m_numchannel = 16;
  m_numnode = num_node;
  m_nakagami = nakagami;
  m_isDay = isDay;
  if (m_isDay){
      MinFadingBias = -10;
      MaxFadingBias = 5;
    }
  else{
      MinFadingBias = -15;
      MaxFadingBias = 10;
    }

  SetFadingBiasValues();
}

LrWpanTschHelper::~LrWpanTschHelper (void)
{
  m_channel->Dispose ();
  m_channel = 0;
}

void
LrWpanTschHelper::SetFadingBiasValues ()
{
  FadingBias.Build(m_numnode, m_numnode, m_numchannel);
  if (m_nakagami)
    {
      m_random = CreateObject<UniformRandomVariable> ();
      m_random->SetAttribute ("Min", DoubleValue ( MinFadingBias));
      m_random->SetAttribute ("Max", DoubleValue (MaxFadingBias));
      for (uint32_t i=0; i<m_numnode; i++)
        for (uint32_t j=0; j<=i; j++)
          for (uint32_t k=0; k<m_numchannel; k++)
            {
              FadingBias[i][j][k] = pow(10, (m_random->GetValue ()/10));
                          FadingBias[i][j][k] = 1;
            }
      for (uint32_t i=0; i<m_numnode; i++)
        for (uint32_t j=i+1; j<m_numnode; j++)
          for (uint32_t k=0; k<m_numchannel; k++)
            {
              FadingBias[i][j][k] = FadingBias[j][i][k];
                          FadingBias[i][j][k] = 1;
            }
    }
  else
    {
      for (uint32_t i=0; i<m_numnode; i++)
        for (uint32_t j=0; j<m_numnode; j++)
          for (uint32_t k=0; k<m_numchannel; k++)
            {
              FadingBias[i][j][k] = 1;
            }
    }
}

void
LrWpanTschHelper::PrintFadingBiasValues(Ptr<OutputStreamWrapper> stream_fadingBias)
{
  //*stream_fadingBias -> GetStream () << "Fading Bias Values for this scenario:" << std::endl;
  for (unsigned char i=0; i<m_numnode; i++)
    {
    for (unsigned char j=0; j<m_numnode; j++)
      {
      for (unsigned char k=0; k<m_numchannel; k++)
        {
          //*stream_fadingBias -> GetStream () << "Bias[" << (int)i << "][" << (int)j << "][" << (int)k << "] = "
          //  << (double)FadingBias[i][j][k] << " ";
          *stream_fadingBias -> GetStream () << 10*log10((double)FadingBias[i][j][k]) << " ";
        }
        //*stream_fadingBias -> GetStream () <<std::endl;
      }
      //*stream_fadingBias -> GetStream () <<std::endl;
    }
}

void
LrWpanTschHelper::EnableLogComponents (void)
{
  LogComponentEnableAll (LOG_PREFIX_TIME);
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  //LogComponentEnable ("LrWpanCsmaCa", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanErrorModel", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanInterferenceHelper", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanTschMac", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanTschNetDevice", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanPhy", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanSpectrumSignalParameters", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanSpectrumValueHelper", LOG_LEVEL_ALL);
}

std::string
LrWpanTschHelper::LrWpanPhyEnumerationPrinter (LrWpanPhyEnumeration e)
{
  switch (e)
    {
    case IEEE_802_15_4_PHY_BUSY:
      return std::string ("BUSY");
    case IEEE_802_15_4_PHY_BUSY_RX:
      return std::string ("BUSY_RX");
    case IEEE_802_15_4_PHY_BUSY_TX:
      return std::string ("BUSY_TX");
    case IEEE_802_15_4_PHY_FORCE_TRX_OFF:
      return std::string ("FORCE_TRX_OFF");
    case IEEE_802_15_4_PHY_IDLE:
      return std::string ("IDLE");
    case IEEE_802_15_4_PHY_INVALID_PARAMETER:
      return std::string ("INVALID_PARAMETER");
    case IEEE_802_15_4_PHY_RX_ON:
      return std::string ("RX_ON");
    case IEEE_802_15_4_PHY_SUCCESS:
      return std::string ("SUCCESS");
    case IEEE_802_15_4_PHY_TRX_OFF:
      return std::string ("TRX_OFF");
    case IEEE_802_15_4_PHY_TX_ON:
      return std::string ("TX_ON");
    case IEEE_802_15_4_PHY_UNSUPPORTED_ATTRIBUTE:
      return std::string ("UNSUPPORTED_ATTRIBUTE");
    case IEEE_802_15_4_PHY_READ_ONLY:
      return std::string ("READ_ONLY");
    case IEEE_802_15_4_PHY_UNSPECIFIED:
      return std::string ("UNSPECIFIED");
    default:
      return std::string ("INVALID");
    }
}

std::string
LrWpanTschHelper::LrWpanMacStatePrinter (LrWpanTschMacState e)
{
  switch (e)
    {
    case TSCH_MAC_IDLE:
      return std::string ("TSCH_MAC_IDLE");
    case TSCH_MAC_CCA:
      return std::string ("TSCH_MAC_CCA");
    case TSCH_MAC_SENDING:
      return std::string ("TSCH_MAC_SENDING");
    case TSCH_MAC_ACK_PENDING:
      return std::string ("TSCH_MAC_ACK_PENDING");
    case TSCH_MAC_ACK_PENDING_END:
      return std::string ("TSCH_MAC_ACK_PENDING_END");
    case TSCH_CHANNEL_ACCESS_FAILURE:
      return std::string ("TSCH_CHANNEL_ACCESS_FAILURE");
    case TSCH_CHANNEL_IDLE:
      return std::string ("TSCH_CHANNEL_IDLE");
    case TSCH_SET_PHY_TX_ON:
      return std::string ("TSCH_SET_PHY_TX_ON");
    case TSCH_MAC_RX:
      return std::string ("TSCH_MAC_RX");
    case TSCH_PKT_WAIT_END:
      return std::string ("TSCH_PKT_WAIT_END");
    default:
      return std::string ("INVALID");
    }
}

void
LrWpanTschHelper::AddMobility (Ptr<LrWpanPhy> phy, Ptr<MobilityModel> m)
{
  phy->SetMobility (m);
}

NetDeviceContainer
LrWpanTschHelper::Install (NodeContainer c)
{
  NetDeviceContainer devices;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); i++)
    {
      Ptr<Node> node = *i;

      Ptr<LrWpanTschNetDevice> netDevice = CreateObject<LrWpanTschNetDevice> ();
      netDevice->SetChannel (m_channel);
      node->AddDevice (netDevice);
      netDevice->SetNode (node);
      // \todo add the capability to change short address, extended
      // address and panId. Right now they are hardcoded in LrWpanTschMac::LrWpanTschMac ()
      devices.Add (netDevice);
    }
  return devices;
}

std::vector<Ptr<LrWpanTschNetDevice> >
LrWpanTschHelper::InstallVector (NodeContainer c)
{
  std::vector<Ptr<LrWpanTschNetDevice> > devices;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); i++)
    {
      Ptr<Node> node = *i;

      Ptr<LrWpanTschNetDevice> netDevice = CreateObject<LrWpanTschNetDevice> ();
      netDevice->SetChannel (m_channel);
      node->AddDevice (netDevice);
      netDevice->SetNode (node);
      // \todo add the capability to change short address, extended
      // address and panId. Right now they are hardcoded in LrWpanTschMac::LrWpanTschMac ()
      devices.push_back (netDevice);
    }
  return devices;
}

EnergySourceContainer
LrWpanTschHelper::InstallEnergySource (NodeContainer c)
{
  LrWpanEnergySourceHelper lrWpanSourceHelper;

  lrWpanSourceHelper.Set ("LrWpanEnergySourceInitialEnergyJ", DoubleValue (2));
  lrWpanSourceHelper.Set ("LrWpanEnergySupplyVoltageV", DoubleValue (1));
  lrWpanSourceHelper.Set ("LrWpanUnlimitedEnergy", BooleanValue (1));

  EnergySourceContainer sources = lrWpanSourceHelper.Install (c);

  return sources;
}

DeviceEnergyModelContainer
LrWpanTschHelper::InstallEnergyDevice (NetDeviceContainer devices, EnergySourceContainer sources)
{
  LrWpanRadioEnergyModelHelper radioEnergyHelper;

  // radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
  DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (devices, sources);

  return deviceModels;
}


int64_t
LrWpanTschHelper::AssignStreams (NetDeviceContainer c, int64_t stream)
{
  int64_t currentStream = stream;
  Ptr<NetDevice> netDevice;
  for (NetDeviceContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      netDevice = (*i);
      Ptr<LrWpanTschNetDevice> lrwpan = DynamicCast<LrWpanTschNetDevice> (netDevice);
      if (lrwpan)
        {
          currentStream += lrwpan->AssignStreams (currentStream);
        }
    }
  return (currentStream - stream);
}

void
LrWpanTschHelper::AssociateToPan (NetDeviceContainer c, uint16_t panId)
{
  NetDeviceContainer devices;
  uint16_t id = 1;
  uint8_t idBuf[2];

  for (NetDeviceContainer::Iterator i = c.Begin (); i != c.End (); i++)
    {
      Ptr<LrWpanTschNetDevice> device = DynamicCast<LrWpanTschNetDevice> (*i);
      if (device)
        {
          idBuf[0] = (id >> 8) & 0xff;
          idBuf[1] = (id >> 0) & 0xff;
          Mac16Address address;
          address.CopyFrom (idBuf);

          device->GetMac ()->SetPanId (panId);
          device->GetMac ()->SetShortAddress (address);
          id++;
        }
    }
  return;
}

static void
PcapSniffLrWpan (Ptr<PcapFileWrapper> file, Ptr<const Packet> packet)
{
  file->Write (Simulator::Now (), packet);
}

void
LrWpanTschHelper::EnablePcapInternal (std::string prefix, Ptr<NetDevice> nd, bool promiscuous, bool explicitFilename)
{
  NS_LOG_FUNCTION (this << prefix << nd << promiscuous << explicitFilename);
  //
  // All of the Pcap enable functions vector through here including the ones
  // that are wandering through all of devices on perhaps all of the nodes in
  // the system.
  //

  // In the future, if we create different NetDevice types, we will
  // have to switch on each type below and insert into the right
  // NetDevice type
  //
  Ptr<LrWpanTschNetDevice> device = nd->GetObject<LrWpanTschNetDevice> ();
  if (device == 0)
    {
      NS_LOG_INFO ("LrWpanTschHelper::EnablePcapInternal(): Device " << device << " not of type ns3::LrWpanTschNetDevice");
      return;
    }

  PcapHelper pcapHelper;

  std::string filename;
  if (explicitFilename)
    {
      filename = prefix;
    }
  else
    {
      filename = pcapHelper.GetFilenameFromDevice (prefix, device);
    }

  Ptr<PcapFileWrapper> file = pcapHelper.CreateFile (filename, std::ios::out,
                                                     PcapHelper::DLT_IEEE802_15_4);

  if (promiscuous == true)
    {
      device->GetMac ()->TraceConnectWithoutContext ("PromiscSniffer", MakeBoundCallback (&PcapSniffLrWpan, file));

    }
  else
    {
      device->GetMac ()->TraceConnectWithoutContext ("Sniffer", MakeBoundCallback (&PcapSniffLrWpan, file));
    }
}

void
LrWpanTschHelper::EnableAsciiInternal (
  Ptr<OutputStreamWrapper> stream,
  std::string prefix,
  Ptr<NetDevice> nd,
  bool explicitFilename)
{
  uint32_t nodeid = nd->GetNode ()->GetId ();
  uint32_t deviceid = nd->GetIfIndex ();
  std::ostringstream oss;

  Ptr<LrWpanTschNetDevice> device = nd->GetObject<LrWpanTschNetDevice> ();
  if (device == 0)
    {
      NS_LOG_INFO ("LrWpanTschHelper::EnableAsciiInternal(): Device " << device << " not of type ns3::LrWpanTschNetDevice");
      return;
    }

  //
  // Our default trace sinks are going to use packet printing, so we have to
  // make sure that is turned on.
  //
  Packet::EnablePrinting ();

  //
  // If we are not provided an OutputStreamWrapper, we are expected to create
  // one using the usual trace filename conventions and do a Hook*WithoutContext
  // since there will be one file per context and therefore the context would
  // be redundant.
  //
  if (stream == 0)
    {
      //
      // Set up an output stream object to deal with private ofstream copy
      // constructor and lifetime issues.  Let the helper decide the actual
      // name of the file given the prefix.
      //
      AsciiTraceHelper asciiTraceHelper;

      std::string filename;
      if (explicitFilename)
        {
          filename = prefix;
        }
      else
        {
          filename = asciiTraceHelper.GetFilenameFromDevice (prefix, device);
        }

      Ptr<OutputStreamWrapper> theStream = asciiTraceHelper.CreateFileStream (filename);

      // Ascii traces typically have "+", '-", "d", "r", and sometimes "t"
      // The Mac and Phy objects have the trace sources for these
      //

      asciiTraceHelper.HookDefaultReceiveSinkWithoutContext<LrWpanTschNetDevice> (device, "MacRx", theStream);

      device->GetMac ()->TraceConnectWithoutContext ("MacTx", MakeBoundCallback (&AsciiLrWpanTschMacTransmitSinkWithoutContext, theStream));
      device->GetMac ()->TraceConnectWithoutContext ("MacMaxRetries", MakeBoundCallback (&AsciiLrWpanTschMacMaxRetriesSinkWithoutContext, theStream));
      asciiTraceHelper.HookDefaultEnqueueSinkWithoutContext<LrWpanTschNetDevice> (device, "MacTxEnqueue", theStream);
      asciiTraceHelper.HookDefaultDequeueSinkWithoutContext<LrWpanTschNetDevice> (device, "MacTxDequeue", theStream);
      asciiTraceHelper.HookDefaultDropSinkWithoutContext<LrWpanTschNetDevice> (device, "MacTxDrop", theStream);

      return;
    }

  //
  // If we are provided an OutputStreamWrapper, we are expected to use it, and
  // to provide a context.  We are free to come up with our own context if we
  // want, and use the AsciiTraceHelper Hook*WithContext functions, but for
  // compatibility and simplicity, we just use Config::Connect and let it deal
  // with the context.
  //
  // Note that we are going to use the default trace sinks provided by the
  // ascii trace helper.  There is actually no AsciiTraceHelper in sight here,
  // but the default trace sinks are actually publicly available static
  // functions that are always there waiting for just such a case.
  //


  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacRx";
  device->GetMac ()->TraceConnect ("MacRx", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultReceiveSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacTx";
  device->GetMac ()->TraceConnect ("MacTx", oss.str (), MakeBoundCallback (&AsciiLrWpanTschMacTransmitSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacMaxRetries";
  device->GetMac ()->TraceConnect ("MacMaxRetries", oss.str (), MakeBoundCallback (&AsciiLrWpanTschMacMaxRetriesSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacTxEnqueue";
  device->GetMac ()->TraceConnect ("MacTxEnqueue", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultEnqueueSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacTxDequeue";
  device->GetMac ()->TraceConnect ("MacTxDequeue", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultDequeueSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacTxDrop";
  device->GetMac ()->TraceConnect ("MacTxDrop", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultDropSinkWithContext, stream));

}

void
LrWpanTschHelper::EnableEnergyAll(Ptr<OutputStreamWrapper> stream)
{
  NodeContainer n = NodeContainer::GetGlobal ();

  for (NodeContainer::Iterator i = n.Begin (); i != n.End (); ++i)
    {
      Ptr<Node> node = *i;
      for (uint32_t j = 0; j < node->GetNDevices (); ++j)
        {
          EnableEnergyInternal (stream, std::string(), node->GetDevice (j), false);
        }
    }
}

void
LrWpanTschHelper::EnableEnergyAllPhy(Ptr<OutputStreamWrapper> stream, EnergySourceContainer sources)
{
    Ptr<EnergySource> sourcePtr;
    for (EnergySourceContainer::Iterator i = sources.Begin (); i != sources.End (); ++i)
      {
        sourcePtr = (*i);
        Ptr<LrWpanEnergySource> lrWpanSourcePtr = DynamicCast<LrWpanEnergySource> (sourcePtr);
        DeviceEnergyModelContainer devicePerSrcPtr = lrWpanSourcePtr->FindDeviceEnergyModels ("ns3::LrWpanRadioEnergyModel");

        uint32_t nodeid = lrWpanSourcePtr-> GetNode() -> GetId();

        for (uint32_t j = 0; j < devicePerSrcPtr.GetN(); j++)
           {
            std::ostringstream oss;
            oss.str ("");
            oss << "/NodeList/" << nodeid << "/DeviceList/" << j << ":";

            Ptr<LrWpanRadioEnergyModel> lrWpanRadioModelPtr = DynamicCast<LrWpanRadioEnergyModel> (devicePerSrcPtr.Get(j));
            NS_ASSERT (lrWpanRadioModelPtr != NULL);

            lrWpanRadioModelPtr->TraceConnect ("CurrentEnergyState", oss.str(), MakeBoundCallback (&ConsumedEnergyTracing, stream));
          }
      }

}

void
LrWpanTschHelper::EnableReceivePower (Ptr<OutputStreamWrapper> stream_recPower, NodeContainer lrwpanNodes)
{
  for (NodeContainer::Iterator i = lrwpanNodes.Begin (); i != lrwpanNodes.End (); ++i)
    {
      Ptr<Node> node = *i;
      for (uint32_t j = 0; j < node->GetNDevices (); ++j)
        {
          Ptr<LrWpanTschNetDevice> device = node->GetDevice(j)->GetObject<LrWpanTschNetDevice> ();
          device->GetPhy ()->TraceConnectWithoutContext ("PhyLinkInformation", MakeCallback (&LrWpanTschMac::GetPhylinkInformation, device->GetMac()));
          device->GetMac ()->TraceConnectWithoutContext ("MacLinkInformation", MakeBoundCallback (&ReceivedPowerTracing, stream_recPower));
        }
    }
}

void
LrWpanTschHelper::EnableEnergy(Ptr<OutputStreamWrapper> stream,Ptr<NetDevice> dev)
{
  EnableAsciiInternal (stream, std::string(), dev, false);
}


void
LrWpanTschHelper::EnableEnergyInternal (
  Ptr<OutputStreamWrapper> stream,
  std::string prefix,
  Ptr<NetDevice> nd,
  bool explicitFilename)
{
  uint32_t nodeid = nd->GetNode ()->GetId ();
  uint32_t deviceid = nd->GetIfIndex ();
  std::ostringstream oss;

  Ptr<LrWpanTschNetDevice> device = nd->GetObject<LrWpanTschNetDevice> ();
  if (device == 0)
    {
      NS_LOG_INFO ("LrWpanTschHelper::EnableEnergyInternal(): Device " << device << " not of type ns3::LrWpanTschNetDevice");
      return;
    }

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacRxDataTxAck";
  device->GetMac ()->TraceConnect ("MacRxDataTxAck", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacTxData";
  device->GetMac ()->TraceConnect ("MacTxData", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacRxData";
  device->GetMac ()->TraceConnect ("MacRxData", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacTxDataRxAck";
  device->GetMac ()->TraceConnect ("MacTxDataRxAck", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacSleep";
  device->GetMac ()->TraceConnect ("MacSleep", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacIdle";
  device->GetMac ()->TraceConnect ("MacIdle", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacChannelBusy";
  device->GetMac ()->TraceConnect ("MacChannelBusy", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacWaitAck";
  device->GetMac ()->TraceConnect ("MacWaitAck", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacEmptyBuffer";
  device->GetMac ()->TraceConnect ("MacEmptyBuffer", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

}

void
LrWpanTschHelper::AddSlotframe(NetDeviceContainer devs, uint8_t slotframehandle, uint16_t size)
{
  MlmeSetSlotframeRequestParams slotframeRequest;
  slotframeRequest.slotframeHandle = slotframehandle;
  slotframeRequest.Operation = MlmeSlotframeOperation_ADD;
  slotframeRequest.size = size;

  for (NetDeviceContainer::Iterator i = devs.Begin (); i != devs.End (); i++)
    {
      Ptr<LrWpanTschNetDevice> lrDevice = DynamicCast<LrWpanTschNetDevice> (*i);
      lrDevice->GetMac ()->MlmeSetSlotframeRequest (slotframeRequest);
    }
}

void
LrWpanTschHelper::AddSlotframe(Ptr<NetDevice> dev, uint8_t slotframehandle, uint16_t size)
{

  MlmeSetSlotframeRequestParams slotframeRequest;
  slotframeRequest.slotframeHandle = slotframehandle;
  slotframeRequest.Operation = MlmeSlotframeOperation_ADD;
  slotframeRequest.size = size;
  dev->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetSlotframeRequest (slotframeRequest);
}

void
LrWpanTschHelper::ModSlotframe(NetDeviceContainer devs, uint8_t slotframehandle, uint16_t size)
{


  MlmeSetSlotframeRequestParams slotframeRequest;
  slotframeRequest.slotframeHandle = slotframehandle;
  slotframeRequest.Operation = MlmeSlotframeOperation_MODIFY;
  slotframeRequest.size = size;

  for (NetDeviceContainer::Iterator i = devs.Begin (); i != devs.End (); i++)
    {
      Ptr<LrWpanTschNetDevice> lrDevice = DynamicCast<LrWpanTschNetDevice> (*i);
      lrDevice->GetMac ()->MlmeSetSlotframeRequest (slotframeRequest);
    }
}

void
LrWpanTschHelper::AddLink(Ptr<NetDevice> src, Ptr<NetDevice> dst, AddLinkParams params)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_ADD_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;


  //10000000 to transmit
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  linkRequest.linkType = MlmeSetLinkRequestlinkType_NORMAL;
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(dst->GetAddress());
  src->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(src->GetAddress());
  dst->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);
}

void
LrWpanTschHelper::AddLink(NetDeviceContainer devs, u_int32_t srcPos, u_int32_t dstPos, AddLinkParams params, bool sharedLink)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_ADD_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;


  //10000000 to transmit
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  if (sharedLink){
      linkRequest.linkOptions.set(2,1);
  }
  linkRequest.linkType = MlmeSetLinkRequestlinkType_NORMAL;
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(devs.Get(dstPos)->GetAddress());
  //PK: XXX
  linkRequest.linkFadingBias = FadingBias[dstPos][srcPos];
  linkRequest.TxID = srcPos;
  linkRequest.RxID = dstPos;
  devs.Get(srcPos)->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(devs.Get(srcPos)->GetAddress());
  //PK: XXX
  linkRequest.linkFadingBias = FadingBias[dstPos][srcPos];
  linkRequest.TxID = srcPos;
  linkRequest.RxID = dstPos;
  devs.Get(dstPos)->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);
}

void
LrWpanTschHelper::AddAdvLink(NetDeviceContainer devs,u_int32_t senderPos, AddLinkParams params)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_ADD_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;


  //10000000 to transmit
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  linkRequest.linkType = MlmeSetLinkRequestlinkType_ADVERTISING;
  linkRequest.nodeAddr = Mac16Address("ff:ff");
  linkRequest.linkFadingBias = NULL;
  linkRequest.TxID = senderPos;
  linkRequest.RxID = 0;
  devs.Get(senderPos)->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(devs.Get(senderPos)->GetAddress());
  for ( u_int32_t i = 0;i < devs.GetN ();i++)
    {
      if (i != senderPos)
        {
          linkRequest.linkFadingBias = FadingBias[i][senderPos];
          linkRequest.TxID = senderPos;
          linkRequest.RxID = i;
          devs.Get(i)->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);
        }
    }
}

#if 1
void LrWpanTschHelper::AddBcastLinks(NetDeviceContainer devs,AddLinkParams params)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_ADD_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;

  //10000000 to transmit
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  linkRequest.linkOptions.set(2,1);
  linkRequest.linkType = MlmeSetLinkRequestlinkType_ADVERTISING;
  linkRequest.nodeAddr = Mac16Address("ff:ff");
  for ( u_int32_t i = 0;i < devs.GetN ();i++)
    {
          linkRequest.TxID = i;
          linkRequest.RxID = 0;
          devs.Get(i)->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);
    }

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.linkType = MlmeSetLinkRequestlinkType_ADVERTISING;
  linkRequest.nodeAddr = Mac16Address("ff:ff");
  for ( u_int32_t i = 0;i < devs.GetN ();i++)
    {
          linkRequest.TxID = 0;
          linkRequest.RxID = i;
          devs.Get(i)->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);
    }
}
#endif

void
LrWpanTschHelper::ModifyLink(Ptr<NetDevice> src, Ptr<NetDevice> dst, AddLinkParams params)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_MODIFY_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;

  //10000000 to transmit
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  linkRequest.linkType = MlmeSetLinkRequestlinkType_NORMAL;
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(dst->GetAddress());
  src->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(src->GetAddress());
  dst->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);
}

void
LrWpanTschHelper::ModifyLink(NetDeviceContainer devs, u_int32_t srcPos, u_int32_t dstPos, AddLinkParams params, bool sharedLink)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_MODIFY_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;

  //10000000 to transmit
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  if (sharedLink){
      linkRequest.linkOptions.set(2,1);
  }
  linkRequest.linkType = MlmeSetLinkRequestlinkType_NORMAL;
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(devs.Get(dstPos)->GetAddress());
  linkRequest.linkFadingBias = FadingBias[dstPos][srcPos];
  linkRequest.TxID = srcPos;
  linkRequest.RxID = dstPos;
  devs.Get(srcPos)->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(devs.Get(srcPos)->GetAddress());
  linkRequest.linkFadingBias = FadingBias[dstPos][srcPos];
  linkRequest.TxID = srcPos;
  linkRequest.RxID = dstPos;
  devs.Get(dstPos)->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);
}

void
LrWpanTschHelper::DeleteLink(Ptr<NetDevice> src, Ptr<NetDevice> dst, AddLinkParams params)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_DELETE_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;


  //10000000 to transmit
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  linkRequest.linkType = MlmeSetLinkRequestlinkType_NORMAL;
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(dst->GetAddress());
  src->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(src->GetAddress());
  dst->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);
}

void
LrWpanTschHelper::DeleteLink(NetDeviceContainer devs, u_int32_t srcPos, u_int32_t dstPos, AddLinkParams params, bool sharedLink)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_DELETE_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;


  //10000000 to transmit
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  if (sharedLink){
      linkRequest.linkOptions.set(2,1);
  }
  linkRequest.linkType = MlmeSetLinkRequestlinkType_NORMAL;
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(devs.Get(dstPos)->GetAddress());
  linkRequest.linkFadingBias = FadingBias[dstPos][srcPos];
  linkRequest.TxID = srcPos;
  linkRequest.RxID = dstPos;
  devs.Get(srcPos)->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(devs.Get(srcPos)->GetAddress());
  linkRequest.linkFadingBias = FadingBias[dstPos][srcPos];
  linkRequest.TxID = srcPos;
  linkRequest.RxID = dstPos;
  devs.Get(dstPos)->GetObject<LrWpanTschNetDevice> ()->GetMac ()->MlmeSetLinkRequest (linkRequest);
}

void
LrWpanTschHelper::ConfigureSlotframeAllToPan(NetDeviceContainer devs, int empty_timeslots)
{
  //DynamicCast<LrWpanTschNetDevice> (netdev.Get(i))
  int size = 1 + devs.GetN () + empty_timeslots - 1;

  AddSlotframe(devs,m_slotframehandle,size);

  //Add links
  AddLinkParams alparams;
  alparams.slotframeHandle = m_slotframehandle;
  alparams.channelOffset = 0;

  alparams.linkHandle = 0;
  alparams.timeslot = 0;
  AddAdvLink (devs,0,alparams);

  alparams.linkHandle = 1;
  alparams.timeslot = 1;
  AddBcastLinks (devs,alparams);

  for (u_int32_t i = 0; i < devs.GetN ()-1 ; i++)
    {
      alparams.linkHandle = i+2;
      alparams.timeslot = i+2;
      AddLink (DynamicCast<LrWpanTschNetDevice> (devs.Get (i+1)),DynamicCast<LrWpanTschNetDevice> (devs.Get (0)),alparams);
    }
   m_slotframehandle++;
}

void
LrWpanTschHelper::EnableTsch(NetDeviceContainer devs, double start, double duration)
{
  MlmeTschModeRequestParams modeRequest,modeRequestoff;
  modeRequest.TSCHMode = MlmeTschMode_ON;
  modeRequestoff.TSCHMode = MlmeTschMode_OFF;

  for (u_int32_t i = 0;i<devs.GetN ();i++)
    {
      Simulator::Schedule(Seconds(start),&LrWpanTschMac::MlmeTschModeRequest,devs.Get (i)->GetObject<LrWpanTschNetDevice> ()->GetMac(),modeRequest);
      Simulator::Schedule(Seconds(start+duration),
                          &LrWpanTschMac::MlmeTschModeRequest,devs.Get (i)->GetObject<LrWpanTschNetDevice> ()->GetMac(),modeRequestoff);
    }
}

void
LrWpanTschHelper::GenerateTraffic(Ptr<NetDevice> dev, Address dst, int packet_size, double start, double duration, double interval)
{
  double end = start+duration;
  Simulator::Schedule(Seconds(start),&LrWpanTschHelper::SendPacket,this,dev,dst,packet_size,interval,end);
}

void
LrWpanTschHelper::SendPacket(Ptr<NetDevice> dev, Address dst, int packet_size, double interval,double end)
{
  if (Now().GetSeconds() <= end)
    {
      Ptr<Packet> pkt = Create<Packet> (packet_size);
      dev->Send(pkt,dst,0x86DD);
    }

  if (Now().GetSeconds() <= end+interval)
    {
      Simulator::Schedule(Seconds(interval),&LrWpanTschHelper::SendPacket,this,dev,dst,packet_size,interval,end);
    }
}

} // namespace ns3

