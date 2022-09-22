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


#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <random>
#include<thread>
#include <unistd.h>
#include <chrono>
 
using namespace ns3;
 
static uint32_t g_received = 0; 
static uint32_t g_retry = 0;
static double signal=0.0;
static double interference=0.0;
double dbmThreshold = 15;
static double alpha_t = 1.0;
double noisedbm = -233.98;
Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel> ();
Ptr<ConstantPositionMobilityModel> mob1 = CreateObject<ConstantPositionMobilityModel> ();
Ptr<ConstantPositionMobilityModel> mobJ = CreateObject<ConstantPositionMobilityModel> ();
Ptr<snowNetDevice> dev0;
Ptr<snowNetDevice> dev1;
Ptr<const SpectrumValue> m_jammer;
double jammer_pathloss = 0;
double hoppingFreq = 500e6;
double jammerHopTime = 0.1;
TvSpectrumTransmitterHelper tvTransHelper;
 
NS_LOG_COMPONENT_DEFINE ("snowErrorDistancePlot");

double dbm2w(double dbm){
  return pow(10.,(dbm-30)/10);
}

double w2dbm(double dbm){
  return 10*log10(dbm)+30.0;
}

Ptr<SpectrumModel> CreateJammerModel ()
{
  Ptr<SpectrumModel> g_snowSpectrumModel;
  Bands bands;
  
  for (int i = 0; i <= 3200; i++)
    {
      BandInfo bi;
      bi.fl = 470e6 + i * 100e3;
      bi.fh = 470e6 + (i + 1) * 100e3;
      bi.fc = (bi.fl + bi.fh) / 2;
      bands.push_back (bi);
    }
  g_snowSpectrumModel = Create<SpectrumModel> (bands);
  return g_snowSpectrumModel;
}

Ptr<SpectrumValue>
CreateJammerPowerSpectralDensity (double txPower, double centerFreq)
{
  Ptr<SpectrumModel> g_snowSpectrumModel = CreateJammerModel ();

  Ptr<SpectrumValue> txPsd = Create <SpectrumValue> (g_snowSpectrumModel);

  txPower = pow (10., (txPower-30) / 10);

  double txPowerDensity = txPower;
  
  int channel = (centerFreq-470e6)/(100e3);

  int start = channel-30;
  int end = channel+30;

  //NS_LOG_DEBUG("Channel number: " << channel);

  for(int i=start;i<=end;i++){
    (*txPsd)[i] = txPowerDensity; // center
  }

  return txPsd;
}

int8_t
GetNominalTxPowerFromPib (uint8_t phyTransmitPower)
{
  int8_t nominalTxPower = phyTransmitPower & 0x1F;

  // Now check the 6th LSB (the "sign" bit).
  // It's a twos-complement format, so the "sign"
  // bit represents -2^5 = -32.
  if (phyTransmitPower & 0x20)
    {
      nominalTxPower -= 32;
    }
  return nominalTxPower;
}

void createInterference(double txPower, double centerFreq)
{
  //NS_LOG_INFO("intereference function");
  /*if(jammer){
    jammer->ClearSignals();
  }
  
  m_jammer = CreateJammerPowerSpectralDensity(txPower,centerFreq);*/
  snowSpectrumValueHelper psdHelper;
  m_jammer = psdHelper.CreateJammerPowerSpectralDensity(GetNominalTxPowerFromPib (txPower), centerFreq);

  dev0->GetPhy()->AddJamming(m_jammer);

  //jammer = Create<snowInterferenceHelper> (m_jammer->GetSpectrumModel ());
  //jammer->AddSignal(m_jammer);
}

void removePrevInterference(){
  if(m_jammer){
    dev0->GetPhy()->RemoveJamming(m_jammer);
  }
}



static void gainCallback (Ptr<const MobilityModel> txMobility, Ptr<const MobilityModel> rxMobility,
                   double txAntennaGain, double rxAntennaGain, double propagationGain,
                   double pathloss)
{
  
  if(txMobility==mob1 && rxMobility==mob0){
    alpha_t=propagationGain;
    //NS_LOG_DEBUG("tx position "<<txMobility->GetPosition()<<" rx position: "<<rxMobility->GetPosition());
    //double t = dbm2w(alpha_t);
    //NS_LOG_DEBUG("new alpha value "<<t<< "  "<<alpha_t<<" "<<pathloss);
    //NS_LOG_DEBUG(txAntennaGain<<" "<<rxAntennaGain);
  }
  else if(rxMobility==mob0 && txMobility->GetPosition() == mobJ->GetPosition()){
    //double t = dbm2w(alpha_t);
    jammer_pathloss = pathloss;
    //NS_LOG_DEBUG(jammer_pathloss);
    //NS_LOG_DEBUG("other alpha value "<<t<< "  "<<alpha_t<<" "<<pathloss);
    //NS_LOG_DEBUG(txAntennaGain<<" "<<rxAntennaGain);
  }
}

int count=-100;
static void
snowErrorDistanceCallback (McpsDataIndicationParams params, Ptr<Packet> p)
{
  if(count==-100 || params.m_dsn==count+1){
    g_received++;
    count=unsigned(params.m_dsn);
    if(count==255){
      count=-1;
    }
    //NS_LOG_DEBUG("sequence "<<unsigned(params.m_dsn));
  }
}

static void
snowRetryCallback (void)
{
  g_retry++;
}

static void
getInterferenceCallback (double signals, double interferences){
  signal = w2dbm(signals);
  interference = w2dbm(interferences);
  //NS_LOG_DEBUG("signal: "<<signal<<" interference: "<<interference);
}

double transmissionPowerGame(double prev_energy, double jammer, int mode=2){
  double energy;
  
  if(mode==1){
    double alpha = (prev_energy-signal)/prev_energy;
    double beta = (jammer-interference)/jammer;

    energy = alpha/(4*beta);
    return energy;
  }
  //NS_LOG_DEBUG("jammer energy "<<w2dbm(jammer_energy));
  //double beta = ((jammer_energy+noise)*(jammer_energy+noise))/(alpha*prev_energy);
  /*double beta = ((jammer_energy+noise)*(jammer_energy+noise))/(alpha*prev_energy);
  //double beta = (jammer_energy+noise)/jammer;
  NS_LOG_DEBUG("beta: "<<beta<<" alpha: "<<alpha);
  if(noise<=(alpha/2)){
    energy = alpha/(4*beta);
    //energy = (4*beta)/alpha;
  }*/
  /*else if( noise>(alpha/2) && noise<=alpha){
    energy = (noise*noise)/(alpha*beta);
  }
  else if(noise>alpha){
    energy = -1000; //code for not transmission
  }
  if(energy>dbm2w(dbmThreshold)){
    return -2000; //code for hopping
  }*/
  //NS_LOG_DEBUG("jammer energy "<<w2dbm(interference)<<" my energy "<<w2dbm(prev_energy*alpha));
  else{
    double gap = -2.0;
    if((signal+gap) > interference)
    {
      NS_LOG_DEBUG("signal: "<<signal<<" interference: "<<interference);
      NS_LOG_DEBUG("my energy is good enough");
      return prev_energy;
    }
    else{
      energy = (interference - gap)+(prev_energy-signal);
      NS_LOG_DEBUG("signal: "<<signal<<" interference: "<<interference);
      NS_LOG_DEBUG("new signal: "<<energy);
      return energy;
    }
  }
  //return w2dbm(energy);
}

//double jammerTransmissionPowerGame(double alpha,)
void setTxPower(double txPower, double centerF){
  snowSpectrumValueHelper svh;
  Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity (txPower, centerF);
  dev1->GetPhy ()->SetTxPowerSpectralDensity (psd);
}

void setFreq(double centerF){
  snowPibAttributeIdentifier id = centerFreq;
  snowPhyPibAttributes attribute;
  attribute.centerFreq = centerF;
  dev0->GetPhy ()->PlmeSetAttributeRequest(id, &attribute);
  dev1->GetPhy ()->PlmeSetAttributeRequest(id, &attribute);
}

int randomNumber (int start, int end){
  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_int_distribution<std::mt19937::result_type> dist6(start,end);
  return dist6(rng);
}

void findBS(void)
{
  auto begin = std::chrono::high_resolution_clock::now();
  while(1){
    usleep(jammerHopTime * 1000000);
    int n=randomNumber(0,52);
    double searchF = (n*6 + 470)*1e6;
    if(searchF==hoppingFreq){
      tvTransHelper.SetAttribute ("StartFrequency", DoubleValue (searchF-3e6));
      auto end = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
      NS_LOG_DEBUG("Time to find BS: "<<elapsed.count() * 1e-9);
      return;
    }
  }

}

double hoppingGame(double currentFreq){
  //randomly selects one channel;
  int curChannel = (currentFreq-470e6)/6;
  int newChannel = randomNumber(0,52);
  if(newChannel==curChannel){
    newChannel=randomNumber(0,52);
  }
  //probe the selected channel by receiving packet from a dummy node
  NS_LOG_DEBUG("hopping to "<<newChannel);
  double newCenterF = (newChannel*6 + 470)*1e6;
  NS_LOG_DEBUG("new frequency "<<newCenterF);
  setFreq(newCenterF);
  setTxPower(-20.0,newCenterF);
  //if prr is above threshold for beacon time, establish the network
  return newCenterF;
}

int main (int argc, char *argv[])
{
  std::ostringstream os;
  std::ofstream berfile ("snow-psr-distance.plt");
  std::ofstream berfile1 ("snow-retry-distance.plt");
 
  //int minDistance = 300;
  //int maxDistance = 1100;  // meters
  int increment = 1;
  int maxPackets = 1000;
  int packetSize = 20;
  double txPower = -20.0;
  double centerF = 500e6;
  double minJammingPower = -20.0;
  double maxJammingPower = 1000.0;
  mobJ->SetPosition (Vector (700.0,0,0));
  CommandLine cmd (__FILE__);
 
  cmd.AddValue ("txPower", "transmit power (dBm)", txPower);
  cmd.AddValue ("packetSize", "packet (MSDU) size (bytes)", packetSize);
  cmd.AddValue ("centerF", "channel number", centerF);
 
  cmd.Parse (argc, argv);
 
  os << "Packet (MSDU) size = " << packetSize << " bytes; tx power = " << txPower << " dBm; channel = " << centerF;
 
  Gnuplot psrplot = Gnuplot ("snow-psr-distance.eps");
  Gnuplot2dDataset psrdataset ("snow-psr-vs-jamming-power");
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
  nodePositionList->Add (Vector (700.0, 0.0, 0.0));
  mobility.SetPositionAllocator (nodePositionList);
  //mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (tvTransmitterNodes);
  // TV transmitter setup
  tvTransHelper.SetChannel (channel);
  tvTransHelper.SetAttribute ("StartFrequency", DoubleValue (497e6));
  tvTransHelper.SetAttribute ("ChannelBandwidth", DoubleValue (6e6));
  tvTransHelper.SetAttribute ("StartingTime", TimeValue (Seconds (0)));
  tvTransHelper.SetAttribute ("TransmitDuration", TimeValue (Minutes (100.0)));
  // 22.22 dBm/Hz from 1000 kW ERP transmit power, flat 6 MHz PSD spectrum assumed for this approximation
  tvTransHelper.SetAttribute ("BasePsd", DoubleValue (minJammingPower));
  tvTransHelper.SetAttribute ("TvType", EnumValue (TvSpectrumTransmitter::TVTYPE_8VSB));
  tvTransHelper.SetAttribute ("Antenna", StringValue ("ns3::IsotropicAntennaModel"));

 
  Ptr<Node> n0 = CreateObject <Node> ();
  Ptr<Node> n1 = CreateObject <Node> ();
  dev0 = CreateObject<snowNetDevice> ();
  dev1 = CreateObject<snowNetDevice> ();
  dev0->SetAddress (Mac16Address ("00:01"));
  dev1->SetAddress (Mac16Address ("00:02"));
  

  dev0->SetChannel (channel);
  dev1->SetChannel (channel);
  n0->AddDevice (dev0);
  n1->AddDevice (dev1);
  dev0->GetPhy ()->SetMobility (mob0);
  dev1->GetPhy ()->SetMobility (mob1);

  setFreq(centerF);
  /*snowSpectrumValueHelper svh;
  Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity (txPower, centerF);
  dev1->GetPhy ()->SetTxPowerSpectralDensity (psd);*/
 
  McpsDataIndicationCallback cb0;
  cb0 = MakeCallback (&snowErrorDistanceCallback);
  dev0->GetMac ()->SetMcpsDataIndicationCallback (cb0);

  McpsDataRetryCallback cb1;
  cb1 = MakeCallback (&snowRetryCallback);
  dev1->GetMac ()->SetMcpsDataRetryCallback (cb1);

  InterferenceCallback cb2;
  cb2 = MakeCallback (&getInterferenceCallback);
  dev0->GetPhy()->SetInterferenceCallback(cb2);
 
  McpsDataRequestParams params;
  params.m_srcAddrMode = SHORT_ADDR;
  params.m_dstAddrMode = SHORT_ADDR;
  params.m_dstPanId = 0;
  params.m_dstAddr = Mac16Address ("00:01");
  params.m_msduHandle = 0;
  params.m_txOptions = 1;
  
  mob0->SetPosition (Vector (0,0,0));
  mob1->SetPosition (Vector (500.0,0,0));
  channel->TraceConnectWithoutContext("Gain",MakeCallback(&gainCallback));

  //createInterference(100.0,centerF);
  //tvTransHelper.InstallAdjacent (tvTransmitterNodes);

  Ptr<Packet> p;
  int gameMode = 1;
  NS_LOG_DEBUG(hoppingFreq);
  int jumpLoop = -1;
  for (int j = minJammingPower; j <= maxJammingPower;  )
    {
      //NS_LOG_DEBUG("jammer path loss "<<jammer_pathloss);
      if(gameMode==1){
        if(j>minJammingPower){
          double newTx = transmissionPowerGame(txPower, j-1);
          if(newTx<=15.0){
            txPower = newTx;
            setTxPower(txPower,centerF);
            NS_LOG_DEBUG("new transmission power:"<<newTx);
          }
          else{
            gameMode=2;
            NS_LOG_DEBUG("Adopting hopping game");
          }
        }
      }

      if(gameMode==2){
        hoppingFreq = hoppingGame(hoppingFreq);
        jumpLoop = j;
        gameMode=3; //have hopped
        txPower = -20;
      }

      if(gameMode==3 && j==jumpLoop+2){
        std::thread th1(findBS);
        th1.detach();
        gameMode=0; //is searching
      }
      
      NS_LOG_DEBUG("jamming power: "<<j);

      for (int i = 0; i < maxPackets; i++)
        {
          p = Create<Packet> (packetSize);
          Simulator::Schedule (Seconds (i),
                               &snowMac::McpsDataRequest,
                               dev1->GetMac (), params, p);
        }
      tvTransHelper.InstallAdjacent (tvTransmitterNodes);
      Simulator::Run ();
      //NS_LOG_DEBUG ("Received " << g_received << " packets for distance " << j);
      //NS_LOG_DEBUG ("Retry " << g_retry << " packets for distance " << j);
      psrdataset.Add (j, (g_received / 1000.0)*100.0);
      if(g_received<500.0 && gameMode==0){
        gameMode=2; //need to hop
      }
      retrydataset.Add (j, txPower);
      g_received = 0;
      g_retry = 0;
      j += increment;
      if(gameMode==1){
        tvTransHelper.SetAttribute ("BasePsd", DoubleValue (j));
      }
      //mob1->SetPosition (Vector (j,0,0));
    }

  //transmission power variation of jammer. show effect on throughput, prr, energy consumption


  //nodes will also increase power and play game. show effect for these as above

  //bs will move to new channel and show effect for above parameters

  //gnuplot
  psrplot.AddDataset (psrdataset);
 
  psrplot.SetTitle (os.str ());
  psrplot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  psrplot.SetLegend ("Jamming power (dBm)", "Packet Success Rate (PSR)");
  psrplot.SetExtra  ("set xrange [-20:1000]\n\
set yrange [0:100]\n\
set grid\n\
set style line 1 linewidth 5\n\
set style increment user");
  psrplot.GenerateOutput (berfile);
  berfile.close ();

  retryplot.AddDataset (retrydataset);
 
  retryplot.SetTitle (os.str ());
  retryplot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  retryplot.SetLegend ("distance (m)", "Packet Retry Rate (PRR)");
  retryplot.SetExtra  ("set xrange [-20:1000]\n\
set yrange [-20:15]\n\
set grid\n\
set style line 1 linewidth 5\n\
set style increment user");
  retryplot.GenerateOutput (berfile1);
  berfile1.close ();
 
  Simulator::Destroy ();
  return 0;
}
