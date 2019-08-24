/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 Fraunhofer ESK
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
 * Author: Vignesh Babu <ns3-dev@esk.fraunhofer.de>
 */

#ifndef LTE_TEST_HANDOVER_FAILURE_H
#define LTE_TEST_HANDOVER_FAILURE_H


#include <ns3/test.h>
#include <ns3/nstime.h>
#include <ns3/node-container.h>
#include <ns3/net-device-container.h>
#include <ns3/vector.h>
#include <ns3/lte-ue-rrc.h>
#include <vector>


namespace ns3 {

class LteUeNetDevice;

}

using namespace ns3;


/**
 * \brief Test suite for
 *
 * \sa ns3::LteHandoverFailureTestCase
 */
class LteHandoverFailureTestSuite : public TestSuite
{
public:
  LteHandoverFailureTestSuite ();
};

/**
 * \ingroup lte
 *
 * \brief Testing the cell reselection procedure by UE at IDLE state
 */
class LteHandoverFailureTestCase : public TestCase
{
public:

  /**
   * \brief Creates an instance of the radio link failure test case.
   * \param name name of this test
   * \param nUes number of UEs
   * \param isIdealRrc if true, simulation uses Ideal RRC protocol, otherwise
   *                   simulation uses Real RRC protocol
   * \param handoverJoiningTimeoutDuration duration of the handover joining timer
   * \param handoverLeavingTimeoutDuration duration of the handover leaving timer
   * \param ueHandoverFailureList list of UEs for which handover fails
   * \param simulationTime duration of the simulation
   */
  LteHandoverFailureTestCase (std::string name, uint32_t nUes, bool isIdealRrc,
                              Time handoverJoiningTimeoutDuration, Time handoverLeavingTimeoutDuration,
                              std::vector<uint32_t> ueHandoverFailureList, Time simulationTime);

  virtual ~LteHandoverFailureTestCase ();

private:

  /**
   * \brief Setup the simulation according to the configuration set by the
   *        class constructor, run it, and verify the result.
   */
  virtual void DoRun ();

  /**
   * Check connected function
   * \param ueDevice the UE device
   * \param enbDevice the ENB device
   */
  void CheckConnected (Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice);

  /**
   * Check if the UE is in idle state
   * \param ueDevice the UE device
   * \param enbDevices the ENB devices
   */
  void CheckIdle (Ptr<NetDevice> ueDevice, NetDeviceContainer enbDevices);

  /**
   * \brief State transition callback function
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   * \param oldState the old state
   * \param newState the new state
   */
  void UeStateTransitionCallback (std::string context, uint64_t imsi,
                                uint16_t cellId, uint16_t rnti,
                                LteUeRrc::State oldState, LteUeRrc::State newState);

  /**
   * \brief Connection established at UE callback function
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   */
  void ConnectionEstablishedUeCallback (std::string context, uint64_t imsi,
                                      uint16_t cellId, uint16_t rnti);

  /**
   * \brief Connection established at eNodeB callback function
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   */
  void ConnectionEstablishedEnbCallback (std::string context, uint64_t imsi,
                                      uint16_t cellId, uint16_t rnti);

  /**
   * \brief This callback function is executed when UE context is removed at eNodeB
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   */
  void ConnectionReleaseAtEnbCallback (std::string context, uint64_t imsi,
                                       uint16_t cellId, uint16_t rnti);

  /**
   * \brief This callback function is executed when UE receives handover command
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the source cell ID
   * \param rnti the RNTI
   * \param targetCellId the target cell ID
   */
  void HandoverStartUeCallback (std::string context, uint64_t imsi,
                              uint16_t cellId, uint16_t rnti, uint16_t targetCellId);

  /**
   * \brief This callback function is executed when random access is successful
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   */
  void HandoverEndOkUeCallback (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti);

  /**
   * \brief This callback function is executed when source eNodeB receives HandoverRequestAck
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the source cell ID
   * \param rnti the RNTI
   * \param targetCellId the target cell ID
   */
  void HandoverStartEnbCallback (std::string context, uint64_t imsi, uint16_t cellId,
                                 uint16_t rnti, uint16_t targetCellId);

  /**
   * \brief This callback function is executed when target eNodeB sends UE context release
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   */
  void HandoverEndOkEnbCallback (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti);

  /**
   * \brief This callback function is executed when handover failure occurs due to random access failure
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   */
  void HandoverEndErrorUeCallback (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti);

  /**
   * \brief This callback function is executed when random access fails at UE
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   */
  void RandomAccessErrorUeCallback (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti);

  /**
   * \brief This callback function is executed when handover fails at eNodeB such as
   *        due to max non-contention based preambles reached
   * \param context the context string
   * \param imsi the IMSI
   * \param rnti the RNTI
   * \param cellId the cell ID
   */
  void HandoverFailureEnbCallback (std::string context, uint64_t imsi, uint16_t rnti, uint16_t cellId,
                                   std::string cause);

  /**
   * \brief This callback function is executed when any of the timers at eNodeB expires
   * \param context the context string
   * \param imsi the IMSI
   * \param rnti the RNTI
   * \param cellId the cell ID
   * \param cause information regarding which timer at the eNodeB expired
   */
  void EnbTimerExpiryCallback (std::string context, uint64_t imsi,
                                uint16_t rnti, uint16_t cellId, std::string cause);

  uint32_t m_nUes; ///<number of UEs
  bool m_isIdealRrc;///< whether the LTE is configured to use ideal RRC
  Time m_handoverJoiningTimeoutDuration; ///<  handoverJoiningTimeout Duration
  Time m_handoverLeavingTimeoutDuration;///<  handoverLeavingTimeout Duration
  std::vector<uint32_t> m_ueHandoverFailureList; ///< list of UEs for which handover will fail
  Time m_simulationTime; ///< simulation duration

  /// The current UE RRC state.
  LteUeRrc::State m_lastState;

}; // end of class LteHandoverFailureTestCase

#endif /* LTE_TEST_HANDOVER_FAILURE_H */
