/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
 /*
  * Copyright (c) 2014 University of Washington
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
  *
  */
 
 #include <iostream>
 #include <stdlib.h>
 #include <ns3/core-module.h>
 #include <ns3/mobility-module.h>
 #include <ns3/spectrum-helper.h>
 #include <ns3/spectrum-analyzer-helper.h>
 #include <ns3/tv-spectrum-transmitter-helper.h>
 
 using namespace ns3;
 
 int main (int argc, char** argv)
 {
   CommandLine cmd;
   cmd.Parse (argc, argv);
   
   /* nodes and positions */
   NodeContainer tvTransmitterNodes;
   NodeContainer spectrumAnalyzerNodes;
   NodeContainer allNodes;
   tvTransmitterNodes.Create (1);
   spectrumAnalyzerNodes.Create (1);
   allNodes.Add (tvTransmitterNodes);
   allNodes.Add (spectrumAnalyzerNodes);
   MobilityHelper mobility;
   Ptr<ListPositionAllocator> nodePositionList = CreateObject<ListPositionAllocator> ();
   nodePositionList->Add (Vector (500.0, 0.0, 0.0)); // TV Transmitter 1; 500 m away from spectrum analyzer
   //nodePositionList->Add (Vector (0.0, 200.0, 0.0)); // TV Transmitter 2; 24 km away from spectrum analyzer
   nodePositionList->Add (Vector (0.0, 0.0, 0.0));  // Spectrum Analyzer
   mobility.SetPositionAllocator (nodePositionList);
   mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
   mobility.Install (allNodes);
 
   /* channel and propagation */
   SpectrumChannelHelper channelHelper = SpectrumChannelHelper::Default ();
   channelHelper.SetChannel ("ns3::MultiModelSpectrumChannel");
   // constant path loss added just to show capability to set different propagation loss models
   // FriisSpectrumPropagationLossModel already added by default in SpectrumChannelHelper
   channelHelper.AddSpectrumPropagationLoss ("ns3::ConstantSpectrumPropagationLossModel");
   Ptr<SpectrumChannel> channel = channelHelper.Create ();
   
   /* TV transmitter setup */
   TvSpectrumTransmitterHelper tvTransHelper;
   tvTransHelper.SetChannel (channel);
   tvTransHelper.SetAttribute ("StartFrequency", DoubleValue (445e6));
   tvTransHelper.SetAttribute ("ChannelBandwidth", DoubleValue (100e3));
   tvTransHelper.SetAttribute ("StartingTime", TimeValue (Seconds (0)));
   tvTransHelper.SetAttribute ("TransmitDuration", TimeValue (Seconds (0.2)));
   // -10 dBm/Hz from 1000 kW ERP transmit power, flat 100 KHz PSD spectrum assumed for this approximation
   tvTransHelper.SetAttribute ("BasePsd", DoubleValue (-10));
   tvTransHelper.SetAttribute ("TvType", EnumValue (TvSpectrumTransmitter::TVTYPE_8VSB));
   tvTransHelper.SetAttribute ("Antenna", StringValue ("ns3::IsotropicAntennaModel"));
   tvTransHelper.InstallAdjacent (tvTransmitterNodes);
 
   /* frequency range for spectrum analyzer */
   std::vector<double> freqs;
   for (int i = 0; i < 60; i+=1)
     {
       freqs.push_back ((i + 4420) * 1e5);
     }
   Ptr<SpectrumModel> spectrumAnalyzerFreqModel = Create<SpectrumModel> (freqs);
 
   /* spectrum analyzer setup */
   SpectrumAnalyzerHelper spectrumAnalyzerHelper;
   spectrumAnalyzerHelper.SetChannel (channel);
   spectrumAnalyzerHelper.SetRxSpectrumModel (spectrumAnalyzerFreqModel);
   spectrumAnalyzerHelper.SetPhyAttribute ("NoisePowerSpectralDensity", DoubleValue (1e-15)); // -120 dBm/Hz
   spectrumAnalyzerHelper.EnableAsciiAll ("spectrum-analyzer-tv-sim");
   NetDeviceContainer spectrumAnalyzerDevices = spectrumAnalyzerHelper.Install (spectrumAnalyzerNodes);
 
   Simulator::Stop (Seconds (0.4));
 
   Simulator::Run ();
 
   Simulator::Destroy ();
 
   std::cout << "simulation done!" << std::endl;
   std::cout << "see spectrum analyzer output file" << std::endl;
 
   return 0;
 }
