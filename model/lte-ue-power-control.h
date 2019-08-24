/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 Piotr Gawlowicz
 * Copyright (c) 2015, University of Padova, Dep. of Information Engineering, SIGNET lab.
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
 * Author: Piotr Gawlowicz <gawlowicz.p@gmail.com>
 *
 * Modified by Michele Polese <michele.polese@gmail.com>
 *    (support for RACH realistic model)
 *
 */

#ifndef LTE_UE_POWER_CONTROL_H
#define LTE_UE_POWER_CONTROL_H

#include <ns3/ptr.h>
#include <ns3/traced-callback.h>
#include <ns3/object.h>
#include <vector>


namespace ns3 {

/**
 * \brief This class realizes Uplink Power Control functionality
 *
 * When LteUePhy is about sending PUSCH/PUCCH/SRS it should ask
 * LteUePowerControl for current channel TX power level and then
 * use it while creating SpectrumValue for Uplink Transmission
 *
 * LteUePowerControl computes TX power level for PUSCH and SRS.
 * PUCCH is realized in ideal way and PUSCH do not use any resources,
 * so there is no need to compute power for that channel
 *
 * LteUePowerControlcomputes TX power based on some preconfigured
 * parameters and current Path-loss. Path-loss is computed as difference
 * between current RSRP and referenceSignalPower level. Current RSRP
 * is passed to LteUePowerControl by LteUePhy. referenceSignalPower is
 * configurable by attribute system
 *
 * Moreover, LteUePhy pass all received TPC values to LteUePowerControl,
 * what is a part of Closed Loop Power Control functionality
 */

class LteUePowerControl : public Object
{
public:

  LteUePowerControl ();
  virtual ~LteUePowerControl ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  // inherited from Object
  virtual void DoInitialize (void);
  virtual void DoDispose (void);

  /**
   * \brief Set PC maximum function
   *
   * \param value the PC maximum value
   */
  void SetPcmax (double value);
  /**
   * \brief Get PC maximum function
   *
   * \returns the PC maximum value
   */
  double GetPcmax ();

  /**
   * \brief Set transmit power function
   *
   * \param value the transmit power value
   */
  void SetTxPower (double value);
  /**
   * \brief Configure reference signal power function
   *
   * \param referenceSignalPower the reference signal power
   */
  void ConfigureReferenceSignalPower (int8_t referenceSignalPower);

  /**
   * \brief Set the cell ID function
   *
   * \param cellId the cell ID
   */
  void SetCellId (uint16_t cellId);
  /**
   * \brief Set the RNTI function
   *
   * \param rnti the RNTI
   */
  void SetRnti (uint16_t rnti);

  /**
   * \brief Set PO nominal PUSCH function
   *
   * \param value the value to set
   */
  void SetPoNominalPusch (int16_t value);
  /**
   * \brief Set PO UE PUSCH function
   *
   * \param value the value to set
   */
  void SetPoUePusch (int16_t value);
  /**
   * \brief Set alpha function
   *
   * \param value the alpha value to set
   */
  void SetAlpha (double value);

  /**
   * \brief Set RSRP function
   *
   * \param value the RSRP value to set
   */
  void SetRsrp (double value);
  /**
   * \brief Set RSRP function
   *
   * \param tpc the TPC to report
   */
  void ReportTpc (uint8_t tpc);

  /// Calculate PUSCH transmit power function
  void CalculatePuschTxPower ();
  void CalculatePrachTxPower ();
  /// Calculate PUCCH transmit power function
  void CalculatePucchTxPower ();
  /// Calculate SRS transmit power function
  void CalculateSrsTxPower ();

  /**
   * \brief Get PUSCH transmit power function
   *
   * \param rb the DL RB list
   * \returns the PUSCH transmit power
   */
  double GetPuschTxPower (std::vector <int> rb);

  /**
   * See 3GPP TS 36.213 6.1, 36.321 5.1.3
   * 
   */
  double GetPrachTxPower (std::vector <int> rb, int32_t preambleReceivedTargetPower);
  /**
   * \brief Get PUCCH transmit power function
   *
   * \param rb unused
   * \returns the PUCCH transmit power
   */
  double GetPucchTxPower (std::vector <int> rb);

  /**
   * \brief Get SRS transmit power function
   *
   * \param rb the DL RB list
   * \returns the SRS transmit power
   */
  double GetSrsTxPower (std::vector <int> rb);

  /**
   * TracedCallback signature for uplink transmit power.
   *
   * \param [in] cellId Cell identifier.
   * \param [in] rnti The C-RNTI identifying the UE.
   * \param [in] power The current TX power.
   */
  typedef void (* TxPowerTracedCallback)
    (uint16_t cellId, uint16_t rnti, double power);

private:
  /**
   * Set subchannel mask function
   *
   * \param [in] mask the subchannel mask
   */
  void SetSubChannelMask (std::vector <int> mask);

  double m_txPower; ///< transmit power
  double m_Pcmax; ///< PC maximum
  double m_Pcmin; ///< PC minimum

  double m_curPuschTxPower; ///< current PUSCH transmit power
  double m_curPrachTxPower;
  double m_curPucchTxPower; ///< current PUCCH transmit power
  double m_curSrsTxPower; ///< current SRS transmit power

  double m_referenceSignalPower; ///< reference signal power
  bool m_rsrpSet; ///< is RSRP set?
  double m_rsrp; ///< RSRP value

  std::vector<int16_t> m_PoNominalPusch; ///< PO nominal PUSCH
  std::vector<int16_t> m_PoUePusch; ///< PO US PUSCH

  int16_t m_PsrsOffset; ///< PSRS offset

  uint16_t m_M_Pusch; ///< size of DL RB list
  uint16_t m_M_Prach;
  int32_t m_preambleReceivedTargetPower;
  std::vector<double> m_alpha; ///< alpha values
  double m_pathLoss; ///< path loss value
  double m_deltaTF; ///< delta TF

  std::vector<int8_t> m_deltaPusch; ///< delta PUSCH
  double m_fc; ///< FC

  uint16_t m_srsBandwidth; ///< SRS bandwidth

  bool m_closedLoop; ///< is closed loop
  bool m_accumulationEnabled; ///< accumulation enabled

  uint16_t m_cellId; ///< cell ID
  uint16_t m_rnti; ///< RNTI
  /**
   * Trace information regarding Uplink TxPower
   * uint16_t cellId, uint16_t rnti, double txPower
   */
  TracedCallback<uint16_t, uint16_t, double> m_reportPuschTxPower;

  TracedCallback<uint16_t, uint16_t, double> m_reportPrachTxPower;
  /**
   * Trace information regarding Uplink TxPower
   * uint16_t cellId, uint16_t rnti, double txPower
   */
  TracedCallback<uint16_t, uint16_t, double> m_reportPucchTxPower;
  /**
   * Trace information regarding Uplink TxPower
   * uint16_t cellId, uint16_t rnti, double txPower
   */
  TracedCallback<uint16_t, uint16_t, double> m_reportSrsTxPower;

};


}

#endif /* LTE_UE_POWER_CONTROL_H */
