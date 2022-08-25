/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
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
 * Author: Tom Henderson <thomas.r.henderson@boeing.com>
 */
 
// This program produces a gnuplot file that plots the packet success rate
// as a function of distance for the snow models, assuming a default
// LogDistance propagation loss model, the 2.4 GHz OQPSK error model, a
// default transmit power of 0 dBm, and a default packet size of 20 bytes of
// snow payload.
#include <ns3/test.h>
#include <ns3/log.h>
#include <ns3/callback.h>
#include <ns3/packet.h>
#include <ns3/simulator.h>
#include <ns3/snow-error-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/snow-net-device.h>
#include <ns3/spectrum-value.h>
#include <ns3/snow-spectrum-value-helper.h>
#include <ns3/snow-mac.h>
#include <ns3/node.h>
#include <ns3/net-device.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/mac16-address.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/uinteger.h>
#include <ns3/nstime.h>
#include <ns3/abort.h>
#include <ns3/command-line.h>
#include <ns3/gnuplot.h>
#include <ns3/mobility-module.h>
#include <ns3/spectrum-helper.h>
#include <ns3/tv-spectrum-transmitter-helper.h>
#include <ns3/spectrum-analyzer-helper.h>
#include <ns3/core-module.h>



 
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <stdlib.h>
 
using namespace ns3;
 
static uint32_t g_received = 0; 
static uint32_t g_retry = 0; 
 
NS_LOG_COMPONENT_DEFINE ("snowErrorDistancePlot");
 
static void
snowErrorDistanceCallback (McpsDataIndicationParams params, Ptr<Packet> p)
{
  g_received++;
}

static void
snowRetryCallback (void)
{
  g_retry++;
}
 
int main (int argc, char *argv[])
{
  std::ostringstream os;
  std::ofstream berfile ("snow-psr-distance.plt");
  std::ofstream berfile1 ("snow-retry-distance.plt");
 
  int minDistance = 1800;
  int maxDistance = 2100;  // meters
  int increment = 1;
  int maxPackets = 1000;
  int packetSize = 20;
  double txPower = 0;
  double centerFreq = 500e6;
 
  CommandLine cmd (__FILE__);
 
  cmd.AddValue ("txPower", "transmit power (dBm)", txPower);
  cmd.AddValue ("packetSize", "packet (MSDU) size (bytes)", packetSize);
  cmd.AddValue ("centerFreq", "channel number", centerFreq);
 
  cmd.Parse (argc, argv);
 
  os << "Packet (MSDU) size = " << packetSize << " bytes; tx power = " << txPower << " dBm; channel = " << centerFreq;
 
  Gnuplot psrplot = Gnuplot ("snow-psr-distance.eps");
  Gnuplot2dDataset psrdataset ("snow-psr-vs-distance");
  Gnuplot retryplot = Gnuplot ("snow-retry-distance.eps");
  Gnuplot2dDataset retrydataset ("snow-retry-vs-distance");

  Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel> ();
  Ptr<LogDistancePropagationLossModel> model = CreateObject<LogDistancePropagationLossModel> ();
  model->SetPathLossExponent (3.2);
  model->SetReference (1, 7.7);
  channel->AddPropagationLossModel (model);

  NodeContainer tvTransmitterNodes;
  tvTransmitterNodes.Create (1);
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> nodePositionList = CreateObject<ListPositionAllocator> ();
  nodePositionList->Add (Vector (500.0, 0.0, 0.0));
  mobility.SetPositionAllocator (nodePositionList);
  //mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (tvTransmitterNodes);
  // TV transmitter setup
  TvSpectrumTransmitterHelper tvTransHelper;
  tvTransHelper.SetChannel (channel);
  tvTransHelper.SetAttribute ("StartFrequency", DoubleValue (497e6));
  tvTransHelper.SetAttribute ("ChannelBandwidth", DoubleValue (6e6));
  tvTransHelper.SetAttribute ("StartingTime", TimeValue (Seconds (0)));
  tvTransHelper.SetAttribute ("TransmitDuration", TimeValue (Minutes (100.0)));
  // 22.22 dBm/Hz from 1000 kW ERP transmit power, flat 6 MHz PSD spectrum assumed for this approximation
  tvTransHelper.SetAttribute ("BasePsd", DoubleValue (-10));
  tvTransHelper.SetAttribute ("TvType", EnumValue (TvSpectrumTransmitter::TVTYPE_8VSB));
  tvTransHelper.SetAttribute ("Antenna", StringValue ("ns3::IsotropicAntennaModel"));

 
  Ptr<Node> n0 = CreateObject <Node> ();
  Ptr<Node> n1 = CreateObject <Node> ();
  Ptr<snowNetDevice> dev0 = CreateObject<snowNetDevice> ();
  Ptr<snowNetDevice> dev1 = CreateObject<snowNetDevice> ();
  dev0->SetAddress (Mac16Address ("00:01"));
  dev1->SetAddress (Mac16Address ("00:02"));
  

  dev0->SetChannel (channel);
  dev1->SetChannel (channel);
  n0->AddDevice (dev0);
  n1->AddDevice (dev1);
  Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel> ();
  dev0->GetPhy ()->SetMobility (mob0);
  Ptr<ConstantPositionMobilityModel> mob1 = CreateObject<ConstantPositionMobilityModel> ();
  dev1->GetPhy ()->SetMobility (mob1);
 
  snowSpectrumValueHelper svh;
  Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity (txPower, centerFreq);
  dev0->GetPhy ()->SetTxPowerSpectralDensity (psd);
 
  McpsDataIndicationCallback cb0;
  cb0 = MakeCallback (&snowErrorDistanceCallback);
  dev1->GetMac ()->SetMcpsDataIndicationCallback (cb0);

  McpsDataRetryCallback cb1;
  cb1 = MakeCallback (&snowRetryCallback);
  dev0->GetMac ()->SetMcpsDataRetryCallback (cb1);
 
  McpsDataRequestParams params;
  params.m_srcAddrMode = SHORT_ADDR;
  params.m_dstAddrMode = SHORT_ADDR;
  params.m_dstPanId = 0;
  params.m_dstAddr = Mac16Address ("00:02");
  params.m_msduHandle = 0;
  params.m_txOptions = 1;
 
  Ptr<Packet> p;
  mob0->SetPosition (Vector (0,0,0));
  mob1->SetPosition (Vector (minDistance,0,0));
  for (int j = minDistance; j < maxDistance;  )
    {
      for (int i = 0; i < maxPackets; i++)
        {
          p = Create<Packet> (packetSize);
          Simulator::Schedule (Seconds (i),
                               &snowMac::McpsDataRequest,
                               dev0->GetMac (), params, p);
        }
      tvTransHelper.InstallAdjacent (tvTransmitterNodes);
      Simulator::Run ();
      NS_LOG_DEBUG ("Received " << g_received << " packets for distance " << j);
      NS_LOG_DEBUG ("Retry " << g_retry << " packets for distance " << j);
      psrdataset.Add (j, g_received / 1000.0);
      retrydataset.Add (j, g_retry / 1000.0);
      g_received = 0;
      g_retry = 0;
      j += increment;
      mob1->SetPosition (Vector (j,0,0));
    }
 
  psrplot.AddDataset (psrdataset);
 
  psrplot.SetTitle (os.str ());
  psrplot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  psrplot.SetLegend ("distance (m)", "Packet Success Rate (PSR)");
  psrplot.SetExtra  ("set xrange [1800:2100]\n\
set yrange [0:1]\n\
set grid\n\
set style line 1 linewidth 5\n\
set style increment user");
  psrplot.GenerateOutput (berfile);
  berfile.close ();

  retryplot.AddDataset (retrydataset);
 
  retryplot.SetTitle (os.str ());
  retryplot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  retryplot.SetLegend ("distance (m)", "Packet Retry Rate (PRR)");
  retryplot.SetExtra  ("set xrange [1800:2100]\n\
set yrange [0:3]\n\
set grid\n\
set style line 1 linewidth 5\n\
set style increment user");
  retryplot.GenerateOutput (berfile1);
  berfile1.close ();
 
  Simulator::Destroy ();
  return 0;
}