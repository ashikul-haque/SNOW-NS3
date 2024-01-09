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
#include <thread>
#include <unistd.h>
#include <chrono>
#include <algorithm>
#include <ctime>
#include <map>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("sim");

static int g_received = 0;
static int g_retry = 0;
double snow_signal = 0.0;
double interference = 0.0;
double dbmThreshold = 15;
//static double alpha_t = 1.0;
double noisedbm = -233.98;
Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel> ();
Ptr<ConstantPositionMobilityModel> mob1 = CreateObject<ConstantPositionMobilityModel> ();
Ptr<ConstantPositionMobilityModel> mobJ = CreateObject<ConstantPositionMobilityModel> ();
Ptr<snowNetDevice> dev0;
Ptr<snowNetDevice> dev1;
Ptr<const SpectrumValue> m_jammer;
double jammer_pathloss = 0;
double hoppingFreq = 500e6;
double jammerHopTime = 0.01;
int transmission = 1;
int jammerWaitTime = 10;
int bsWaitTime = 20;
bool jammerFound = false;
double jammerEnergymw = 72e6; //20 wh

int snowTxInterval = 1;
double txPower = 0.0;
double minJammingPower = 17.0;
int nSnowNodes = 100;
double nodeBW = 100e3;
TvSpectrumTransmitterHelper tvTransHelper;

bool Q = true;
int tr = 16;
int training_time = tr*3600;
double simulationTime = 4*3600;
// Output control
std::string Qfile = "Qtable7.txt";
std::string Jfile = "Jtable7.txt";
bool print = true;
bool save = true;
bool load = true;
bool training = true;
bool retrain = false;
double epsilon=0.95;
double epsilon_2=0.01; //after training 
int actionNum = 53*26; //channel*tx 

int currAction=-1;
int currJction=-1;

/**helpers for Qlearning**/

struct action{
  int channel;
  double tx;
  double utility;
  //double bw;
};

//std::map<int, action> actions;

std::vector <action> Qtable;
std::vector <action> Jtable;
//std:: vector <action> Qactions;
Ptr<UniformRandomVariable> Qrv = CreateObject<UniformRandomVariable> ();

//int findMaxElement(std::vector <action> table);

/*helper function to initialize the Qtable not used in this version*/

void initQ () {

  for(int txp=-10;txp<=15;txp++){
    for(int chan=0;chan<=52;chan++){
      action temp;
      temp.tx=txp;
      temp.channel=chan;
      temp.utility=0;
      Qtable.push_back(temp);
    }
  }
  NS_LOG_DEBUG("Q table initialized "<<Qtable.size());
  std::cout<<Qtable.size()<<"\n";
}

void initJ () {

  for(int txp=0;txp<=25;txp++){
    for(int chan=0;chan<=52;chan++){
      action temp;
      temp.tx=txp;
      temp.channel=chan;
      temp.utility=0;
      Jtable.push_back(temp);
    }
  }
  std::cout<<Jtable.size()<<"\n";
}

void
loadQ ()
{
  std::ifstream file (Qfile);
  // Read the next line from File until it reaches the
  // end.
  double t;
  int c = 0;
  action t1;
  while (file >> t)
  {
    // Now keep reading next line
    // and push it in vector function until end of file
    c++;
    if (c == 1)
    {
      t1.channel = (int) t;
    }
    else if (c == 2)
    {
      t1.tx = t;
    }
    else if (c == 3)
    {
      t1.utility = t;
      Qtable.push_back (t1);
      c = 0;
    }
  }
  NS_LOG_DEBUG ("Q table loaded " << Qtable.size ());
}

void
loadJ ()
{
  std::ifstream file (Jfile);
  // Read the next line from File until it reaches the
  // end.
  double t;
  int c = 0;
  action t1;
  while (file >> t)
  {
    // Now keep reading next line
    // and push it in vector function until end of file
    c++;
    if (c == 1)
    {
      t1.channel = (int) t;
    }
    else if (c == 2)
    {
      t1.tx = t;
    }
    else if (c == 3)
    {
      t1.utility = t;
      Jtable.push_back (t1);
      c = 0;
    }
  }
  NS_LOG_DEBUG ("J table loaded " << Jtable.size ());
}

//trace functions

double gama = 0.5;
//int reward=0;

int
randomNumber (int start, int end)
{
  std::random_device dev;
  std::mt19937 rng (dev ());
  std::uniform_int_distribution<std::mt19937::result_type> dist6 (start, end);
  return dist6 (rng);
}

/*function to update the Qtable*/
void
Qtable_update (double reward)
{
  if (currAction != -1)
  {
    Qtable.at (currAction).utility += gama * (reward - Qtable.at (currAction).utility);
  }
}

void
Jtable_update (double reward)
{
  if (currJction != -1)
  {
    Jtable.at (currJction).utility += gama * (reward - Jtable.at (currJction).utility);
  }
}

void
ChangeEpsilon ()
{
  std::cout << "changing epsilon \n";
  epsilon = epsilon_2;
}

/*function to find the maximum Qvalue and return the Qtable index*/
int
findMaxElement (std::vector<action> table, int start = 0, bool hop = false)
{

  double max = -99999;
  int index = 0;
  if (!hop)
  {
    for (int i = 0; i < table.size (); i++)
    {
      if (table.at (i).utility > max)
        {
          max = table.at (i).utility;
          index = i;
        }
    }
  }
  else
  {
    for (int i = start; i < start+53; i++)
    {
      if (table.at (i).utility > max)
        {
          max = table.at (i).utility;
          index = i;
        }
    }
  }

  return index;
}

void
setTxPower (double txPower, int channel)
{
  double centerF = (channel * 6 + 470) * 1e6;
  snowSpectrumValueHelper svh;
  Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity (txPower, centerF);
  dev1->GetPhy ()->SetTxPowerSpectralDensity (psd);
}

void
setFreq (int channel)
{
  double centerF = (channel * 6 + 470) * 1e6;
  snowPibAttributeIdentifier id = centerFreq;
  snowPhyPibAttributes attribute;
  attribute.centerFreq = centerF;
  dev0->GetPhy ()->PlmeSetAttributeRequest (id, &attribute);
  dev1->GetPhy ()->PlmeSetAttributeRequest (id, &attribute);
}

void play() {
  double random = Qrv->GetValue(0,1);
  int chosen_action;
  int num_entries = actionNum;
  
 
  if(random<epsilon) 
  { 
   
   chosen_action= Qrv->GetInteger(0,num_entries-1);
   //chosen_action= randomNumber(0,num_entries-1);
   currAction = chosen_action;
    
   }
   else 
  {
    chosen_action =  findMaxElement(Qtable);
    currAction = chosen_action;
  }
  //change channel
  setFreq (Qtable.at(currAction).channel);
  //change tx
  setTxPower (Qtable.at(currAction).tx, Qtable.at(currAction).channel);
  //return to caller
  return;
  //caller will do tx

}

void jPlay(int start=0, int end=actionNum) {

  double random = Qrv->GetValue(0,1);
  int chosen_action;
  
  if(random<epsilon || load) 
  { 
    chosen_action= Qrv->GetInteger(start, end-1);
    //chosen_action= randomNumber(start, end-1);
    currJction = chosen_action;
  }

  else 
  {
    if(start!=0 && end!=actionNum){
      chosen_action = findMaxElement(Jtable, start, true);
    }
    else{
      chosen_action = findMaxElement(Jtable);
    }
    currJction = chosen_action;
  }
  return;

}

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
  return 10 * log10 (1000*dbm);
}

/*static void
gainCallback (Ptr<const MobilityModel> txMobility, Ptr<const MobilityModel> rxMobility,
              double txAntennaGain, double rxAntennaGain, double propagationGain, double pathloss)
{

  if (txMobility == mob1 && rxMobility == mob0)
    {
      alpha_t = propagationGain;
      //NS_LOG_DEBUG("tx position "<<txMobility->GetPosition()<<" rx position: "<<rxMobility->GetPosition());
      //double t = dbm2w(alpha_t);
      //NS_LOG_DEBUG("new alpha value "<<t<< "  "<<alpha_t<<" "<<pathloss);
      //NS_LOG_DEBUG(txAntennaGain<<" "<<rxAntennaGain);
    }
  else if (rxMobility == mob0 && txMobility->GetPosition () == mobJ->GetPosition ())
    {
      //double t = dbm2w(alpha_t);
      jammer_pathloss = pathloss;
      //NS_LOG_DEBUG(jammer_pathloss);
      //NS_LOG_DEBUG("other alpha value "<<t<< "  "<<alpha_t<<" "<<pathloss);
      //NS_LOG_DEBUG(txAntennaGain<<" "<<rxAntennaGain);
    }
}*/

int count = -100;
static void
snowErrorDistanceCallback (McpsDataIndicationParams params, Ptr<Packet> p)
{
  if (count == -100 || params.m_dsn == count + 1)
    {
      g_received++;
      count = unsigned (params.m_dsn);
      if (count == 255)
        {
          count = -1;
        }
      //NS_LOG_DEBUG("sequence "<<unsigned(params.m_dsn));
    }
}

static void
snowRetryCallback (void)
{
  g_retry++;
}

/*static void
getInterferenceCallback (double signals, double interferences)
{
  snow_signal = w2dbm (signals);
  interference = w2dbm (interferences);
  //NS_LOG_DEBUG("signal: "<<signal<<" interference: "<<interference);
}*/

void
doTransmission (int intTime, McpsDataRequestParams params, int packetSize, int snowTxInterval = 60)
{
  Ptr<Packet> p;
  for (int i = 0; i < nSnowNodes; i++)
    {
      p = Create<Packet> (packetSize);
      Simulator::Schedule (Seconds (intTime+snowTxInterval+i*.1), &snowMac::McpsDataRequest, dev1->GetMac (), params,
                                   p);
    }
  
}

//double jammerTransmissionPowerGame(double alpha,)
double util_h = 2;

void
findBS (void)
{
  jammerFound = false;
  jPlay ();
  int bsChannel = Qtable.at (currAction).channel;
  //double bsTX = Qtable.at(currAction).tx;
  util_h = 2;
  //double util_s = 0;
  int n = Jtable.at (currJction).channel;
  double jammerTX = Jtable.at (currJction).tx;
  int s = currJction - n; //to find tx start index in jtable
  int e = s + 53;
  tvTransHelper.SetAttribute ("BasePsd", DoubleValue (jammerTX));
  double searchF = (n * 6 + 470) * 1e6; //that channels frequency
  //auto begin = std::chrono::high_resolution_clock::now ();

  while (1)
    {
      usleep (jammerHopTime * 1000000); //time needed for hopping
      //NS_LOG_INFO ("Jammer is in freq: " << searchF);
      /*if (n == bsChannel)
        {
          jammerFound = true;
          //NS_LOG_INFO ("Jammer is in BS channel");
        }*/
      if (n == bsChannel)
      {
        jammerFound = true;
        //std::cout<<"jammer found bs\n";
        //NS_LOG_INFO ("Jammer has detected BS and jamming");
        tvTransHelper.SetAttribute ("StartFrequency", DoubleValue (searchF - 3e6));

        if(!training){
          double max = -99999;
          int index = currJction;
          for (int i = currJction; i < Jtable.size (); i += 53)
          {
            if (Jtable.at (i).utility > max)
              {
                max = Jtable.at (i).utility;
                index = i;
              }
          }
          std::cout<<Jtable.at(index).channel<<" "<< Jtable.at(currJction).channel<< " " << Jtable.at(index).tx<<" "<< Jtable.at(currJction).tx<<std::endl;
          currJction = index;
          tvTransHelper.SetAttribute ("BasePsd", DoubleValue (Jtable.at(currJction).tx));
        }
        
        return;
      }
      jPlay (s, e);
      //std::cout<<"Jammer channel: "<<Jtable.at(currJction).channel<<" "<<bsChannel<<"Jammer tx: "<<Jtable.at(currJction).tx<<"\n";
      n = Jtable.at (currJction).channel; //selecting a channel randomly for hopping
      searchF = (n * 6 + 470) * 1e6; //that channels frequency
      util_h += 2;
    }
}

void
saveT ()
{
  NS_LOG_DEBUG ("Saving");
  std::fstream file;
  file.open (Qfile, std::ios_base::out);

  for (int i = 0; i < Qtable.size (); i++)
    {
      file << Qtable.at (i).channel << std::endl;
      file << Qtable.at (i).tx << std::endl;
      file << Qtable.at (i).utility << std::endl;
    }

  file.close ();

  std::fstream file2;
  file2.open (Jfile, std::ios_base::out);

  for (int i = 0; i < Jtable.size (); i++)
    {
      file2 << Jtable.at (i).channel << std::endl;
      file2 << Jtable.at (i).tx << std::endl;
      file2 << Jtable.at (i).utility << std::endl;
    }

  file2.close ();
}

/*void
findTX (void)
{
  double max = Jtable.at(currJction).utility;
  int index = 0;
  for (int i = currJction; i < Jtable.size (); i+=53)
    {
      //debug
      if(Jtable.at(i).channel!=Jtable.at(currJction).channel){
        NS_LOG_DEBUG("ERROR IN FINDTX");
      }

      if (Jtable.at (i).utility > max)
        {
          max = Jtable.at (i).utility;
          index = i;
        }
    }
  
}*/

int
main (int argc, char *argv[])
{
  std::ostringstream os;
  int packetSize = 40;
  //double centerF = 500e6;
  mobJ->SetPosition (Vector (700.0, 0, 0));
  CommandLine cmd;

  cmd.AddValue ("interval", "packet interval", snowTxInterval);
  cmd.AddValue ("nodes", "number of nodes", nSnowNodes);
  cmd.Parse (argc, argv);

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
  // Jammer setup
  tvTransHelper.SetChannel (channel);
  tvTransHelper.SetAttribute ("StartFrequency", DoubleValue (470e6));
  tvTransHelper.SetAttribute ("ChannelBandwidth", DoubleValue (6e6));
  tvTransHelper.SetAttribute ("StartingTime", TimeValue (Seconds (0)));
  tvTransHelper.SetAttribute ("TransmitDuration", TimeValue (Minutes (100.0)));
  // 22.22 dBm/Hz from 1000 kW ERP transmit power, flat 6 MHz PSD spectrum assumed for this approximation
  tvTransHelper.SetAttribute ("BasePsd", DoubleValue (17.0));
  tvTransHelper.SetAttribute ("TvType", EnumValue (TvSpectrumTransmitter::TVTYPE_8VSB));
  tvTransHelper.SetAttribute ("Antenna", StringValue ("ns3::IsotropicAntennaModel"));

  Ptr<Node> n0 = CreateObject<Node> ();
  Ptr<Node> n1 = CreateObject<Node> ();
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

  setFreq (0);
  setTxPower(5, 0);
  /*snowSpectrumValueHelper svh;
  Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity (txPower, centerF);
  dev1->GetPhy ()->SetTxPowerSpectralDensity (psd);*/

  McpsDataIndicationCallback cb0;
  cb0 = MakeCallback (&snowErrorDistanceCallback);
  dev0->GetMac ()->SetMcpsDataIndicationCallback (cb0);

  McpsDataRetryCallback cb1;
  cb1 = MakeCallback (&snowRetryCallback);
  dev1->GetMac ()->SetMcpsDataRetryCallback (cb1);

  /*InterferenceCallback cb2;
  cb2 = MakeCallback (&getInterferenceCallback);
  dev0->GetPhy ()->SetInterferenceCallback (cb2);*/

  McpsDataRequestParams params;
  params.m_srcAddrMode = SHORT_ADDR;
  params.m_dstAddrMode = SHORT_ADDR;
  params.m_dstPanId = 0;
  params.m_dstAddr = Mac16Address ("00:01");
  params.m_msduHandle = 0;
  params.m_txOptions = 1;

  mob0->SetPosition (Vector (0, 0, 0));
  mob1->SetPosition (Vector (690.0, 0, 0));
  //channel->TraceConnectWithoutContext ("Gain", MakeCallback (&gainCallback));

  //createInterference(100.0,centerF);
  //tvTransHelper.InstallAdjacent (tvTransmitterNodes);
  NS_LOG_DEBUG("setup complete");
  if(load && Q){
    NS_LOG_DEBUG("Loading");
    loadQ();
    loadJ();
    if (!retrain)
    {
      training = false;
      ChangeEpsilon ();
      save = false;
    }
  }
  else if(Q){
    NS_LOG_DEBUG("Initializing");
    initQ();
    initJ();
  }
  Ptr<Packet> p;
  auto timer = std::chrono::high_resolution_clock::now ();
  double epp = 0;
  int usedTime = -1;
  double total_received = 0;
  double total_sent = 0;
  double total_retry = 0;
  double c_received = 0;
  double c_sent = 0;
  //int j = minJammingPower;
  //std::ofstream logfile;
  //logfile.open ("log.csv",std::ios_base::app);
  //intial game start
  if(Q){
    play();
    std::thread th1(findBS);
    th1.detach();
  }
  auto begin = std::chrono::high_resolution_clock::now ();
  //int c=0;

  while (1)
  {
      g_received = 0;
      g_retry = 0;
      auto timer_end = std::chrono::high_resolution_clock::now ();
      auto timer_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds> (timer_end - timer);
      double time_count = timer_elapsed.count () * 1e-9;
      int intTime = (int) time_count;
      if(intTime >= training_time && training){
        ChangeEpsilon();
        if(save){
          saveT();
        }
        training = false;
      }
      if (intTime % snowTxInterval == 0 && intTime > usedTime)
        {
          //if(jammerFound) c++;
          doTransmission (intTime, params, packetSize, snowTxInterval);
          tvTransHelper.InstallAdjacent (tvTransmitterNodes);
          Simulator::Run ();
          /*if(g_received>nSnowNodes){
            g_received = nSnowNodes;
          }*/
          //double rng = Qrv->GetValue(0,1);
          //g_received *= rng;
          //NS_LOG_DEBUG("received: "<<g_received<<", "<<g_retry);
          c_sent+=nSnowNodes;
          if(g_received > nSnowNodes){
            g_received = nSnowNodes;
          }
          c_received+=g_received;
          if (!training)
            {
              total_sent += nSnowNodes;
              if (g_received > nSnowNodes)
                {
                  g_received = nSnowNodes;
                }
              total_received += g_received;
              total_retry += g_retry;
              epp += dbm2mw (Qtable.at(currAction).tx) * (g_retry + nSnowNodes);
              //logfile <<total_received << ","<< total_retry << "," << total_sent << ","  << epp << "\n" ;
            }
          //NS_LOG_INFO ("Received: " << g_received << " sent: " << nSnowNodes);
          //NS_LOG_INFO ("Retry: " << g_retry);

          //NS_LOG_INFO ("current time before sending: " << intTime);
          //NS_LOG_INFO ("jamming power: " << j);
          //determine tx,channel from game function
          if(jammerFound && Q){
            //std::cout<<"Jammer found or packet is getting lost"<<std::endl;
            std::cout<<Qtable.at(currAction).channel<<" "<<Qtable.at(currAction).tx<<" "<<Qtable.at(currAction).utility<<"\n";
            std::cout<<Jtable.at(currJction).channel<<" "<<Jtable.at(currJction).tx<<" "<<Jtable.at(currJction).utility<<"\n";
            
              //NS_LOG_DEBUG("Packet dropped or jammer found");
              auto timer_end = std::chrono::high_resolution_clock::now ();
              auto timer_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds> (timer_end - begin);
              time_count = timer_elapsed.count () * 1e-9; // tx time for this channel
              double snr = w2dbm(dbm2w(Qtable.at(currAction).tx)/(dbm2w(Jtable.at(currJction).tx)+dbm2w(noisedbm))) - (Qtable.at(currAction).tx+10);
              //double rng = Qrv->GetValue(0,1);
              double sinr = w2dbm(dbm2w(Qtable.at(currAction).tx)/(dbm2w(Jtable.at(currJction).tx)+dbm2w(noisedbm)));
              double channelReward = time_count;
              double reward;
              if(training){
                reward = 0.2*(snr/15) + 0.2*(channelReward/10) + 0.6*((c_received/c_sent + 1)/2);
              }
              else{
                reward = 0.2*(snr/15) + 0.2 +0.6*((c_received/c_sent + 1)/2);
              }
              
              //logfile <<Qtable.at(currAction).tx<<","<<Jtable.at(currJction).tx<<","<<snr << ","<< time_count << "," << reward << Jtable.at(currJction).utility <<"\n" ;
              //std::cout<<"snr test: "<<w2dbm(dbm2w(Qtable.at(currAction).tx)/(dbm2w(Jtable.at(currJction).tx)+dbm2w(noisedbm)))<<std::endl;
              Qtable_update(reward);
              snr = -w2dbm(dbm2w(Qtable.at(currAction).tx)/(dbm2w(Jtable.at(currJction).tx)+dbm2w(noisedbm))) - (Jtable.at(currJction).tx+1);
              channelReward = (util_h + time_count);
              if(training){
                reward = 0.3*(snr/15) - 0.1*(channelReward/700) - 0.6*((c_received/c_sent + 1)/2);
              }
              else{
                reward = 0.3*(snr/15) - 0.1 - 0.6*((c_received/c_sent + 1)/2);
              }
              //double reward_j = 0.3*(snr/15) - 0.1*(channelReward/65) - 0.6*(c_received/c_sent);
              //std::cout<<"reward: "<<reward<<"\n";
              Jtable_update(reward);
              std::cout <<"TX: "<< Qtable.at(currAction).tx<<", TX_J: "<<Jtable.at(currJction).tx<<", SNR: "<<sinr << ", Time: "<< time_count <<" received: "<<c_received<<" " << "sent: "<< c_sent <<" "<<g_retry<<"\n" ;
              std::cout<<findMaxElement(Qtable)<<" "<<currAction<<" "<< findMaxElement(Jtable)<<" "<<currJction<<std::endl;
              if(training || (!training && findMaxElement(Qtable)!=currAction && ((c_received/c_sent)< 0.7 || (c_received/c_sent)> 0.92))){
                // && findMaxElement(Jtable)!=currJction
                c_received = 0;
                c_sent = 0;
                play();
                begin = std::chrono::high_resolution_clock::now ();
                std::thread th1(findBS);
                th1.detach();
              }
          }
          /*else if(jammerFound && c>=5){
            c=0;
            std::thread th1(findBS);
            th1.detach();
            jammerFound = false;
          }*/
          //set tx, channel

          //determine jammer tx, channel from jam function

          //set jammer tx, channel

          //tvTransHelper.SetAttribute ("BasePsd", DoubleValue (j));

          //do tx

          //NS_LOG_INFO ("tx completed");

          //when training time is over, start counting
          usedTime = intTime;
        }

      if (intTime >= simulationTime)
        {
          std::cout<<Qtable.at(currAction).channel<<" "<<Qtable.at(currAction).tx<<" "<<Qtable.at(currAction).utility<<"\n";
          std::cout<<Jtable.at(currJction).channel<<" "<<Jtable.at(currJction).tx<<" "<<Jtable.at(currJction).utility<<"\n";
          std::cout <<total_received << ","<< total_retry << "," << total_sent << ","  << epp << "\n" ;
          break;
        }
      //intTime++;
  }

  /*
  ifstream file("file.txt");
 
    vector<string> v;
    string str;
 
    // Read the next line from File until it reaches the
    // end.
    while (file >> str) {
        // Now keep reading next line
        // and push it in vector function until end of file
        v.push_back(str);
    }*/
  //if(Q) Simulator::Schedule(Seconds(training_time), &ChangeEpsilon);
  //logfile.close();
  Simulator::Destroy ();
  return 0;
}
