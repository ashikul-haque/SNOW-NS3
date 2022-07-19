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
 * Author: Gary Pei <guangyu.pei@boeing.com>
 */
#include "snow-spectrum-value-helper.h"
#include <ns3/log.h>
#include <ns3/spectrum-value.h>

#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("snowSpectrumValueHelper");

Ptr<SpectrumModel> g_snowSpectrumModel; //!< Global object used to initialize the snow Spectrum Model

/**
 * \ingroup snow
 * \brief Helper class used to automatically initialize the snow Spectrum Model objects
 */
class snowSpectrumModelInitializer
{
public:
  snowSpectrumModelInitializer (double bandwidth, double centerFreq)
  {
    NS_LOG_FUNCTION (this);

    Bands bands;
    BandInfo bi;
    bi.fl = centerFreq - bandwidth/2;
    bi.fh = centerFreq + bandwidth/2;
    bi.fc = centerFreq;
    bands.push_back (bi);
    g_snowSpectrumModel = Create<SpectrumModel> (bands);
  }

} g_snowSpectrumModelInitializerInstance; //!< Global object used to initialize the snow Spectrum Model

snowSpectrumValueHelper::snowSpectrumValueHelper (void)
{
  NS_LOG_FUNCTION (this);
  m_noiseFactor = 1.0;
}

snowSpectrumValueHelper::~snowSpectrumValueHelper (void)
{
  NS_LOG_FUNCTION (this);
}

Ptr<SpectrumValue>
snowSpectrumValueHelper::CreateTxPowerSpectralDensity (double txPower)
{
  NS_LOG_FUNCTION (this);
  Ptr<SpectrumValue> txPsd = Create <SpectrumValue> (g_snowSpectrumModel);

  // txPower is expressed in dBm. We must convert it into natural unit (W).
  txPower = pow (10., (txPower - 30) / 10);

  // The effective occupied bandwidth of the signal is modelled to be 2 MHz.
  // 99.5% of power is within +/- 1MHz of center frequency, and 0.5% is outside.
  // There are 5 bands containing signal power.  The middle (center) band
  // contains half of the power.  The two inner side bands contain 49.5%.
  // The two outer side bands contain roughly 0.5%.
  double txPowerDensity = txPower / 2.0e6;


  // The channel assignment is in section 6.1.2.1
  // Channel 11 centered at 2.405 GHz, 12 at 2.410 GHz, ... 26 at 2.480 GHz
  
  (*txPsd)[0] = txPowerDensity; // center

  // If more power is allocated to more subbands in future revisions of
  // this model, make sure to renormalize so that the integral of the
  // txPsd still equals txPower

  return txPsd;
}

Ptr<SpectrumValue>
snowSpectrumValueHelper::CreateNoisePowerSpectralDensity ()
{
  NS_LOG_FUNCTION (this);
  Ptr<SpectrumValue> noisePsd = Create <SpectrumValue> (g_snowSpectrumModel);

  static const double BOLTZMANN = 1.3803e-23;
  // Nt  is the power of thermal noise in W
  double Nt = BOLTZMANN * 290.0;
  // noise Floor (W) which accounts for thermal noise and non-idealities of the receiver
  double noisePowerDensity = m_noiseFactor * Nt;

  (*noisePsd)[0] = noisePowerDensity;

  return noisePsd;
}

double
snowSpectrumValueHelper::TotalAvgPower (Ptr<const SpectrumValue> psd)
{
  NS_LOG_FUNCTION (psd);
  double totalAvgPower = 0.0;

  NS_ASSERT (psd->GetSpectrumModel () == g_snowSpectrumModel);

  // numerically integrate to get area under psd using 1 MHz resolution

  
  totalAvgPower = (*psd)[0];
  totalAvgPower *= 1.0e6;

  return totalAvgPower;
}

} // namespace ns3
