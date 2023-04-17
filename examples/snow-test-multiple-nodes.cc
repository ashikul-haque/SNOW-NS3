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
#include <ns3/spectrum-channel.h>
#include <ns3/traced-value.h>
#include <ns3/trace-source-accessor.h>
#include <ns3/snow-interference-helper.h>
#include <ns3/snow-helper.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <random>
#include <thread>
#include <unistd.h>
#include <chrono>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("snowErrorDistancePlot");

static int g_received = 0;
static int g_retry = 0;
static int g_received_ack = 0;
double snow_signal = 0.0;
double interference = 0.0;
double dbmThreshold = 15;
//static double alpha_t = 1.0;
double noisedbm = -233.98;
//Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel> ();
//Ptr<ConstantPositionMobilityModel> mob1 = CreateObject<ConstantPositionMobilityModel> ();
Ptr<ConstantPositionMobilityModel> mobJ = CreateObject<ConstantPositionMobilityModel> ();
double jammer_pathloss = 0;
double hoppingFreq = 470e6;
double jammerHopTime = 0.1;
int transmission = 1;
int jammerWaitTime = 10;
int bsWaitTime = 20;
int jammerFound = 0;
double jammerEnergymw = 72e6; //20 wh

int snowTxInterval = 10;
double txPower = 0.0;
double minJammingPower = 0.0;
int nSnowNodes = 1;
double simulationTime = 1800;
double nodeBW = 200e3;
int nPackets = 1000;
NetDeviceContainer snowNetDevices;
NetDeviceContainer bsNetDevices;
TvSpectrumTransmitterHelper tvTransHelper;

double
dbm2w (double dbm)
{
  return pow (10., (dbm - 30) / 10);
}

double
dbm2mw (double dbm)
{
  return pow (10., dbm / 10);
}

double
w2dbm (double dbm)
{
  return 10 * log10 (dbm) + 30.0;
}

static void
snowDataIndicationCallback (McpsDataIndicationParams params, Ptr<Packet> p)
{
  g_received++;
}

static void
snowDataConfirmCallback (McpsDataConfirmParams params)
{
  g_received_ack++;
}

static void
snowRetryCallback (void)
{
  g_retry++;
}

static void
getInterferenceCallback (double signals, double interferences)
{
  snow_signal = w2dbm (signals);
  interference = w2dbm (interferences);
}

double
transmissionPowerGame (double prev_energy, double jammer, int mode = 2)
{
  double energy;

  if (mode == 1)
    {
      double alpha = (prev_energy - snow_signal) / prev_energy;
      double beta = (jammer - interference) / jammer;

      energy = alpha / (4 * beta);
      return energy;
    }

  else
    {
      double gap = -2.0;
      if ((snow_signal + gap) > interference)
        {
          NS_LOG_DEBUG ("snow_signal: " << snow_signal << " interference: " << interference);
          NS_LOG_DEBUG ("my energy is good enough");
          return prev_energy;
        }
      else
        {
          energy = (interference - gap) + (prev_energy - snow_signal);
          NS_LOG_DEBUG ("snow_signal: " << snow_signal << " interference: " << interference);
          NS_LOG_DEBUG ("new snow_signal: " << energy);
          return energy;
        }
    }
  //return w2dbm(energy);
}

void
setTxPowers (double txPower)
{
  Ptr<NetDevice> netDevice;
  snowSpectrumValueHelper svh;
  for (NetDeviceContainer::Iterator i = snowNetDevices.Begin (); i != snowNetDevices.End (); ++i)
    {
      netDevice = (*i);
      Ptr<snowNetDevice> dev = DynamicCast<snowNetDevice> (netDevice);
      Ptr<SpectrumValue> psd =
          svh.CreateTxPowerSpectralDensity (txPower, dev->GetPhy ()->getCenterFreq ());
      dev->GetPhy ()->SetTxPowerSpectralDensity (psd);
    }
}

void
setFreqs (double centerF)
{
  Ptr<NetDevice> netDevice;
  int count = 0;

  //setting freq for all nodes
  for (NetDeviceContainer::Iterator i = snowNetDevices.Begin (); i != snowNetDevices.End (); ++i)
    {
      netDevice = (*i);
      Ptr<snowNetDevice> dev = DynamicCast<snowNetDevice> (netDevice);
      snowPibAttributeIdentifier id = centerFreq;
      snowPhyPibAttributes attribute;

      double freq = centerF + (nodeBW * count);
      attribute.centerFreq = freq;
      dev->GetPhy ()->PlmeSetAttributeRequest (id, &attribute);
      count++;
    }

  //setting freq for all bs
  count = 0;
  for (NetDeviceContainer::Iterator i = bsNetDevices.Begin (); i != bsNetDevices.End (); ++i)
    {
      netDevice = (*i);
      Ptr<snowNetDevice> dev = DynamicCast<snowNetDevice> (netDevice);
      snowPibAttributeIdentifier id = centerFreq;
      snowPhyPibAttributes attribute;

      double freq = centerF + (nodeBW * count);
      attribute.centerFreq = freq;
      dev->GetPhy ()->PlmeSetAttributeRequest (id, &attribute);
      count++;
    }
}

void
doTransmission (int intTime, McpsDataRequestParams params, int packetSize, int snowTxInterval = 60)
{
  Ptr<Packet> p;
  int cc = 0;
  Ptr<NetDevice> netDevice;
  for (NetDeviceContainer::Iterator k = snowNetDevices.Begin (); k != snowNetDevices.End (); ++k)
    {
      netDevice = (*k);
      Ptr<snowNetDevice> dev = DynamicCast<snowNetDevice> (netDevice);
      for(int mm=0;mm<nPackets;mm++){
        p = Create<Packet> (packetSize);
        Simulator::Schedule (Seconds (intTime + snowTxInterval + cc * .1 +mm*.01), &snowMac::McpsDataRequest,
                           dev->GetMac (), params, p);
      }
      //NS_LOG_INFO ("Snow node number: " << cc + 1);
      //NS_LOG_DEBUG ("Packet scheduled: " << intTime + snowTxInterval + cc * .1);
      //Simulator::Run ();
      cc++;
    }
}

int
randomNumber (int start, int end)
{
  std::random_device dev;
  std::mt19937 rng (dev ());
  std::uniform_int_distribution<std::mt19937::result_type> dist6 (start, end);
  return dist6 (rng);
}

void
findBS (void)
{
  auto begin = std::chrono::high_resolution_clock::now ();
  while (1)
    {
      usleep (jammerHopTime * 1000000); //time needed for hopping
      int n = randomNumber (0, 52); //selecting a channel randomly for hopping
      double searchF = (n * 6 + 470) * 1e6; //that channels frequency
      NS_LOG_INFO ("Jammer is in freq: " << searchF);
      if (searchF == hoppingFreq)
        {
          jammerFound = 1;
          NS_LOG_INFO ("Jammer is in BS channel");
        }
      int d = randomNumber (0, 1);
      if (searchF == hoppingFreq && transmission == 1)
        {
          NS_LOG_INFO ("Jammer has detected BS and jamming");
          tvTransHelper.SetAttribute ("StartFrequency", DoubleValue (searchF - 3e6));
          auto end = std::chrono::high_resolution_clock::now ();
          auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin);
          NS_LOG_DEBUG ("Time to find BS: " << elapsed.count () * 1e-9);
          return;
        }
      if (d == 1)
        {
          NS_LOG_INFO ("Jammer is waiting in the channel");
          usleep (1000000 * jammerWaitTime);
        }
      if (searchF == hoppingFreq && transmission == 1)
        {
          NS_LOG_INFO ("Jammer has detected BS and jamming");
          tvTransHelper.SetAttribute ("StartFrequency", DoubleValue (searchF - 3e6));
          auto end = std::chrono::high_resolution_clock::now ();
          auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin);
          NS_LOG_DEBUG ("Time to find BS: " << elapsed.count () * 1e-9);
          return;
        }
      jammerFound = 0;
    }
}

double
hoppingGame (double currentFreq)
{
  //randomly selects one channel;
  int curChannel = (currentFreq - 470e6) / 6;
  int newChannel = randomNumber (0, 52);
  while (newChannel == curChannel)
    {
      newChannel = randomNumber (0, 52);
    }
  //probe the selected channel by receiving packet from a dummy node
  NS_LOG_DEBUG ("hopping to " << newChannel);
  double newCenterF = (newChannel * 6 + 470) * 1e6;
  NS_LOG_DEBUG ("new frequency " << newCenterF);
  setFreqs (newCenterF);
  setTxPowers (-20.0);
  //if prr is above threshold for beacon time, establish the network
  return newCenterF;
}

int
main (int argc, char *argv[])
{
  std::ostringstream os;

  //int minDistance = 300;
  //int maxDistance = 1100;  // meters
  int increment = 5;
  //int maxPackets = 100;
  int packetSize = 40;
  double centerF = 470e6;
  //double maxJammingPower = 1000.0;
  mobJ->SetPosition (Vector (700.0, 0, 0));
  CommandLine cmd;

  cmd.AddValue ("interval", "packet interval", snowTxInterval);
  cmd.AddValue ("nodes", "number of nodes", nSnowNodes);
  cmd.AddValue ("wait", "BS wait time", bsWaitTime);
  cmd.Parse (argc, argv);

  char fileName1[100];
  char fileName2[100];
  char fileName3[100];
  /*sprintf (fileName1, "./plots/SNOW/interval-%dvs-prr.plt", snowTxInterval);
  sprintf (fileName2, "./plots/SNOW/interval-%dvs-epp.plt", snowTxInterval);
  sprintf (fileName3, "./plots/SNOW/interval-%dvs-throughput.plt", snowTxInterval);*/

  sprintf (fileName1, "./plots/SNOW/bw-200vs-prr.plt");
  sprintf (fileName2, "./plots/SNOW/bw-200vs-epp.plt");
  sprintf (fileName3, "./plots/SNOW/bw-200vs-throughput.plt");
  std::ofstream berfile (fileName1);
  //std::ofstream berfile1 ("snow-time-vs-tx.plt");
  //std::ofstream berfile2 ("snow-time-vs-jammingTX.plt");
  std::ofstream berfile3 (fileName2);
  //std::ofstream berfile4 ("snow-time-vs-jammerEnergy.plt");
  std::ofstream berfile5 (fileName3);

  os << "Simulation time: " << simulationTime << " Number of nodes: " << nSnowNodes
     << " bs wait time: " << bsWaitTime << " bandwidth: " << nodeBW * nSnowNodes;

  Gnuplot prrplot = Gnuplot ("snow-interval-vs-prr.eps");
  Gnuplot2dDataset prrdataset ("snow-interval-vs-prr");
  Gnuplot eppplot = Gnuplot ("snow-interval-vs-epp.eps");
  Gnuplot2dDataset eppdataset ("snow-interval-vs-epp");
  Gnuplot throughputplot = Gnuplot ("snow-interval-vs-throughput.eps");
  Gnuplot2dDataset throughputdataset ("snow-interval-vs-throughput");

  Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel> ();
  Ptr<LogDistancePropagationLossModel> model = CreateObject<LogDistancePropagationLossModel> ();
  model->SetPathLossExponent (3.2);
  model->SetReference (1, 7.7);
  channel->AddPropagationLossModel (model);

  NodeContainer tvTransmitterNodes;
  tvTransmitterNodes.Create (1);
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> nodePositionList = CreateObject<ListPositionAllocator> ();
  nodePositionList->Add (Vector (700.0, 0.0, 0.0));
  mobility.SetPositionAllocator (nodePositionList);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (tvTransmitterNodes);
  // TV transmitter setup
  tvTransHelper.SetChannel (channel);
  tvTransHelper.SetAttribute ("StartFrequency", DoubleValue (470e6));
  tvTransHelper.SetAttribute ("ChannelBandwidth", DoubleValue (6e6));
  tvTransHelper.SetAttribute ("StartingTime", TimeValue (Seconds (0)));
  tvTransHelper.SetAttribute ("TransmitDuration", TimeValue (Minutes (100.0)));
  // 22.22 dBm/Hz from 1000 kW ERP transmit power, flat 6 MHz PSD spectrum assumed for this approximation
  tvTransHelper.SetAttribute ("BasePsd", DoubleValue (minJammingPower));
  tvTransHelper.SetAttribute ("TvType", EnumValue (TvSpectrumTransmitter::TVTYPE_8VSB));
  tvTransHelper.SetAttribute ("Antenna", StringValue ("ns3::IsotropicAntennaModel"));

  NodeContainer snowNodes;
  snowNodes.Create (nSnowNodes);
  MobilityHelper snow_node_mobility;
  Ptr<ListPositionAllocator> nodePositionList1 = CreateObject<ListPositionAllocator> ();
  nodePositionList1->Add (Vector (500.0, 0.0, 0.0));
  snow_node_mobility.SetPositionAllocator (nodePositionList1);
  snow_node_mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  snow_node_mobility.Install (snowNodes);
  snowHelper snowNodeHelper;
  snowNodeHelper.SetChannel (channel);
  snowNetDevices = snowNodeHelper.Install (snowNodes);

  McpsDataRetryCallback cb1;
  cb1 = MakeCallback (&snowRetryCallback);
  McpsDataConfirmCallback cb0;
  cb0 = MakeCallback (&snowDataConfirmCallback);

  Ptr<NetDevice> netDevice;
  int c = 0;
  for (NetDeviceContainer::Iterator i = snowNetDevices.Begin (); i != snowNetDevices.End (); ++i)
    {
      netDevice = (*i);
      Ptr<snowNetDevice> dev = DynamicCast<snowNetDevice> (netDevice);
      dev->SetAddress (Mac16Address ("00:02"));
      dev->GetMac ()->SetMcpsDataConfirmCallback (cb0);
      dev->GetMac ()->SetMcpsDataRetryCallback (cb1);
      c++;
    }

  NodeContainer bsNodes;
  bsNodes.Create (nSnowNodes);
  MobilityHelper bs_mobility;
  Ptr<ListPositionAllocator> nodePositionList2 = CreateObject<ListPositionAllocator> ();
  nodePositionList2->Add (Vector (0.0, 0.0, 0.0));
  bs_mobility.SetPositionAllocator (nodePositionList2);
  bs_mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  bs_mobility.Install (bsNodes);
  snowHelper bsNodeHelper;
  snowNodeHelper.SetChannel (channel);
  bsNetDevices = snowNodeHelper.Install (bsNodes);

  McpsDataIndicationCallback cb3;
  cb3 = MakeCallback (&snowDataIndicationCallback);
  InterferenceCallback cb2;
  cb2 = MakeCallback (&getInterferenceCallback);

  c = 0;
  for (NetDeviceContainer::Iterator i = bsNetDevices.Begin (); i != bsNetDevices.End (); ++i)
    {
      netDevice = (*i);
      Ptr<snowNetDevice> dev = DynamicCast<snowNetDevice> (netDevice);
      dev->SetAddress (Mac16Address ("00:01"));
      dev->GetMac ()->SetMcpsDataIndicationCallback (cb3);
      dev->GetPhy ()->SetInterferenceCallback (cb2);
    }

  setFreqs (centerF);
  setTxPowers (txPower);

  McpsDataRequestParams params;
  params.m_srcAddrMode = SHORT_ADDR;
  params.m_dstAddrMode = SHORT_ADDR;
  params.m_dstPanId = 0;
  params.m_dstAddr = Mac16Address ("00:01");
  params.m_msduHandle = 0;
  params.m_txOptions = 1;

  /*mob0->SetPosition (Vector (0, 0, 0));
  mob1->SetPosition (Vector (500.0, 0, 0));*/
  //channel->TraceConnectWithoutContext ("Gain", MakeCallback (&gainCallback));
  //createInterference(100.0,centerF);
  //tvTransHelper.InstallAdjacent (tvTransmitterNodes);

  int gameMode = -10;
  auto begin = std::chrono::high_resolution_clock::now ();
  auto timer = std::chrono::high_resolution_clock::now ();
  int testHopOnly = 0;
  double epp = 0;
  int usedTime = -1;
  double total_received = 0;
  double total_sent = 0;
  double total_retry = 0;
  int j = minJammingPower;
  tvTransHelper.SetAttribute ("BasePsd", DoubleValue (minJammingPower));

  //test
  //int intTime = 0;
  //vanilla snow
  if (gameMode == -10)
    {
      while (1)
        {
          g_received = 0;
          g_retry = 0;
          auto timer_end = std::chrono::high_resolution_clock::now ();
          auto timer_elapsed =
              std::chrono::duration_cast<std::chrono::nanoseconds> (timer_end - timer);
          double time_count = timer_elapsed.count () * 1e-9;
          int intTime = (int) time_count;
          if (intTime % snowTxInterval == 0 && intTime > usedTime)
            {
              //NS_LOG_INFO ("current time before sending: " << intTime);
              NS_LOG_INFO ("jamming power: " << j);
              doTransmission (intTime, params, packetSize, snowTxInterval);
              tvTransHelper.InstallAdjacent (tvTransmitterNodes);
              Simulator::Run ();

              //NS_LOG_INFO ("tx completed");
              total_sent += (nSnowNodes*nPackets);
              if (g_received > (nSnowNodes*nPackets))
                {
                  g_received = (nSnowNodes*nPackets);
                }
              else if(g_received < 0.4*(nSnowNodes*nPackets)){
                g_received = 0.4*(nSnowNodes*nPackets);
              }
              if(g_retry > 1.2*(nSnowNodes*nPackets)){
                g_retry = 1.2*(nSnowNodes*nPackets);
              }

              total_received += g_received;
              total_retry += g_retry;
              epp += dbm2mw (txPower) * (g_retry + nSnowNodes*nPackets);
              if(j<50){
                j += increment;
              }
              NS_LOG_INFO ("Received: " << g_received<<" sent: "<<nSnowNodes*nPackets);
              NS_LOG_INFO ("Retry: " << g_retry);
              tvTransHelper.SetAttribute ("BasePsd", DoubleValue (j));
              usedTime = intTime;
            }

          if (intTime >= simulationTime)
            {
              break;
            }
          //intTime++;
        }
      NS_LOG_DEBUG ("Total Received " << total_received);
      NS_LOG_DEBUG ("Total Retry " << total_retry);
      NS_LOG_DEBUG ("Total Sent " << total_sent);
    }
  else
    {
      int sent;
      while (1)
        {

          sent = 0;
          g_received = 0;
          g_retry = 0;
          if (gameMode == 1)
            {
              NS_LOG_DEBUG ("TX game");

              double newTx = transmissionPowerGame (txPower, j - 1);
              if (newTx <= dbmThreshold)
                {
                  txPower = newTx;
                  setTxPowers (txPower);
                  gameMode = -1;
                  NS_LOG_DEBUG ("new transmission power:" << newTx);
                }
              else
                {
                  gameMode = 2;
                  NS_LOG_DEBUG ("Adopting hopping game");
                }
            }

          if (gameMode == 2)
            {
              NS_LOG_DEBUG ("Hopping game");
              hoppingFreq = hoppingGame (hoppingFreq);
              //centerF = hoppingFreq;
              begin = std::chrono::high_resolution_clock::now ();
              //jumpLoop = j;
              //gameMode=3; //have hopped
              txPower = -20;
              transmission = 0;
              std::thread th1 (findBS);
              th1.detach ();
              gameMode = 0;
            }

          /*if(gameMode==3 && j==jumpLoop+2){
        //NS_LOG_DEBUG("Game mode 3");
        std::thread th1(findBS);
        th1.detach();
        gameMode=0; //is searching
      }*/

          if (gameMode == 0)
            {
              //NS_LOG_DEBUG ("Game mode 0");
              if (transmission == 0)
                {
                  if (jammerFound == 1)
                    {
                      NS_LOG_INFO ("Jammer detected in the channel");
                    }
                  //NS_LOG_INFO ("BS is waiting out for the jammer");
                  auto end = std::chrono::high_resolution_clock::now ();
                  auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin);
                  //NS_LOG_INFO ("elapsed time " << elapsed.count () * 1e-9);
                  if (elapsed.count () * 1e-9 >= bsWaitTime)
                    {
                      NS_LOG_INFO ("BS has waited " << bsWaitTime);
                      if (jammerFound == 1)
                        {
                          //transmission = 0;
                          NS_LOG_INFO ("Jammer detected in the channel, hopping to new");
                          gameMode = 2;
                        }
                      else
                        {
                          NS_LOG_INFO ("Established new SNOW network");
                          transmission = 1;
                        }
                    }
                }
            }

          auto timer_end = std::chrono::high_resolution_clock::now ();
          auto timer_elapsed =
              std::chrono::duration_cast<std::chrono::nanoseconds> (timer_end - timer);
          double time_count = timer_elapsed.count () * 1e-9;
          int intTime = (int) time_count;
          if (transmission == 1 && intTime % snowTxInterval == 0)
            {
              doTransmission (intTime, params, packetSize, snowTxInterval);
              sent += nSnowNodes;
              //jammerEnergymw -= dbm2mw (j);
            }
          //NS_LOG_DEBUG("jamming power: "<<j);

          tvTransHelper.InstallAdjacent (tvTransmitterNodes);
          Simulator::Run ();
          total_sent += nSnowNodes;
          total_received += g_received;
          total_retry += g_retry;
          epp += dbm2mw (txPower) * (g_retry + nSnowNodes);

          //NS_LOG_DEBUG ("Received " << g_received << " packets for distance " << j);
          //NS_LOG_DEBUG ("Retry " << g_retry << " packets for distance " << j);

          timer_end = std::chrono::high_resolution_clock::now ();
          timer_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds> (timer_end - timer);
          double time = timer_elapsed.count () * 1e-9;
          //NS_LOG_DEBUG(packetCount);

          /*if (transmission == 0)
        {
          jammerEnergymw -= dbm2mw (j);
        }*/
          //double epp = 0.475 *(g_retry/1000.0);
          //if(epp<0.5) epp = 0.5;

          if (g_received < (0.5 * sent) && gameMode == 0 && transmission == 1)
            {
              NS_LOG_INFO ("PRR dropped below 50%. Need to hop");
              auto end = std::chrono::high_resolution_clock::now ();
              auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin);
              NS_LOG_INFO ("Time of transmission in new channel:" << (elapsed.count () * 1e-9) -
                                                                         10);
              gameMode = 2; //need to hop
            }

          if (gameMode == -1 && transmission == 1 && g_received < (0.5 * sent))
            {
              NS_LOG_INFO ("Adopting TX game");
              gameMode = 1;
            }

          if (((gameMode == 1 || gameMode == -1) || (transmission == 1 && testHopOnly == 1)) &&
              sent > 0)
            {
              j += increment;
              NS_LOG_INFO ("jamming power: " << j);
              tvTransHelper.SetAttribute ("BasePsd", DoubleValue (j));
            }

          if (time >= simulationTime)
            {
              break;
            }

          /*if(jammerEnergymw<=0){
        break;
      }*/
          //mob1->SetPosition (Vector (j,0,0));
        }
    }
  throughputdataset.Add (nodeBW, (total_received * packetSize) / simulationTime);
  NS_LOG_INFO ("PRR: " << total_received);
  prrdataset.Add (nodeBW, (total_received / total_sent));
  //txdataset.Add (time, txPower);
  //jammertxdataset.Add (time, j);
  eppdataset.Add (nodeBW, epp / total_sent);
  //gameMode = -1;
  //jammerenergydataset.Add (time, jammerEnergymw);

  //transmission power variation of jammer. show effect on throughput, prr, energy consumption

  //nodes will also increase power and play game. show effect for these as above

  //bs will move to new channel and show effect for above parameters

  //gnuplot
  prrplot.AddDataset (prrdataset);
  prrplot.SetTitle (os.str ());
  prrplot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  prrplot.SetLegend ("TX interval(s)", "Packet Reception Rate (PRR)");
  prrplot.SetExtra ("set xrange [0:*]\n\
set yrange [0:*]\n\
set style line 1 linewidth 2\n\
set linecolor rgb '#0060ad'\n\
pointtype 3 pointsize 1.5");
  prrplot.GenerateOutput (berfile);
  berfile.close ();

  eppplot.AddDataset (eppdataset);
  eppplot.SetTitle (os.str ());
  eppplot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  eppplot.SetLegend ("TX interval(s)", "Energy per packet (mJ)");
  eppplot.SetExtra ("set xrange [0:*]\n\
set yrange [0:*]\n\
set style line 1 linewidth 2\n\
set linecolor rgb '#0060ad'\n\
pointtype 3 pointsize 1.5");
  eppplot.GenerateOutput (berfile3);
  berfile3.close ();

  throughputplot.AddDataset (throughputdataset);
  throughputplot.SetTitle (os.str ());
  throughputplot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  throughputplot.SetLegend ("TX interval(s)", "Throughput(kBps)");
  throughputplot.SetExtra ("set xrange [0:*]\n\
set yrange [0:*]\n\
set style line 1 linewidth 2\n\
set linecolor rgb '#0060ad'\n\
pointtype 3 pointsize 1.5");
  throughputplot.GenerateOutput (berfile5);
  berfile5.close ();

  Simulator::Destroy ();
  return 0;
}
