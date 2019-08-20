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

#ifndef LTE_TEST_UPLINK_SYNCHRONIZATION_H
#define LTE_TEST_UPLINK_SYNCHRONIZATION_H


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
 * \sa ns3::LteUplinkSynchronizationTestCase
 */
class LteUplinkSynchronizationTestSuite : public TestSuite
{
public:
  LteUplinkSynchronizationTestSuite ();
};

/**
 * \ingroup lte
 *
 * \brief Testing the cell reselection procedure by UE at IDLE state
 */
class LteUplinkSynchronizationTestCase : public TestCase
{
public:

  /**
   * Parameters to verify at end of simulation
   */
  struct UeStatus
  {
    bool isUplinkOutOfSync; ///< true if UE loses its uplink synchronization atleast once
    bool isConnected; ////< true if UE is in connected state at end of simulation
  };

  /**
   * \brief Creates an instance of the uplink synchronization test case.
   * \param name name of this test
   * \param isIdealRrc if true, simulation uses Ideal RRC protocol, otherwise
   *                   simulation uses Real RRC protocol
   * \param isDynamicTimerConfiguration if true, uplink timeAlignmentTimer
   *                   value is based on mobility of UE, otherwise the value is fixed
   * \param isDl true if data transfer is in downlink direction, otherwise false
   * \param velocity the velocity of the UE
   * \param interPacketInterval the interval between packet transmission
   * \param applicationStopTime the time at which the application stops sending data
   */
  LteUplinkSynchronizationTestCase (std::string name, bool isIdealRrc,
                              bool isDynamicTimerConfiguration, bool isDl, Vector velocity,
                              double interPacketInterval, Time applicationStopTime,
                              UeStatus status);

  virtual ~LteUplinkSynchronizationTestCase ();

private:
  /**
   * \brief Setup the simulation according to the configuration set by the
   *        class constructor, run it, and verify the result.
   */
  virtual void DoRun ();

  /**
   * Check connected function
   * \param ueDevice the UE device
   * \param enbDevices the ENB device
   */
  void CheckConnected (Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice);

  /**
   * Check if the UE is in idle state
   * \param ueDevice the UE device
   * \param enbDevices the ENB device
   */
  void CheckIdle (Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice);

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
   * \brief This callback function is executed when any of the timers at eNodeB expires
   * \param context the context string
   * \param imsi the IMSI
   * \param rnti the RNTI
   * \param cellId the cell ID
   * \param cause information regarding which timer at the eNodeB expired
   */
  void EnbTimerExpiryCallback (std::string context, uint64_t imsi,
                                uint16_t rnti, uint16_t cellId, std::string cause);

  /**
   * \brief This callback function is executed when time alignment timer expires
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   * \param result indicates if after the time alignment timer expires,
   *               UE is considered to be uplink synchronized or not(ignored in case of handover)
   */
  void UplinkOutofSyncCallback (std::string context, uint64_t imsi,
                                      uint16_t cellId, uint16_t rnti, std::string result);

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
   * \brief This callback function is executed when RACH occurs in connected state
   * \param context the context string
   * \param imsi the IMSI
   * \param rnti the RNTI
   * \param type indicates the type of random access procedure executed when UE is in
   *             connected but uplink out-of-sync state
   */
  void ConnectedOutOfSnycRachCallback (std::string context, uint64_t imsi, uint16_t rnti, std::string type);

  /**
   * \brief This callback function is executed when UE receives timing advance(in RAR or TAC ctrl msg)
   * \param context the context string
   * \param imsi the IMSI
   * \param rnti the RNTI
   * \param timer the time alignment timer value
   * \param cause indicates if the timing advance was received is RAR or TAC ctrl msg
   */
  void TimeAlignmentTimerUpdateCallback (std::string context, uint64_t imsi, uint16_t rnti,
                            Time timer, std::string cause);

  /**
   * \brief Connection Reconfiguration msg received by UE callback function
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   */
  void ConnectionReconfigReceivedCallback (std::string context, uint64_t imsi,
                                           uint16_t cellId, uint16_t rnti);

  /**
   * \brief Connection Reconfiguration Complete msg received by eNodeB callback function
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   */
  void ConnectionReconfigCompleteReceivedCallback (std::string context, uint64_t imsi,
                                              uint16_t cellId, uint16_t rnti);


  bool m_isIdealRrc; ///< whether the LTE is configured to use ideal RRC
  bool m_isDynamicTimerConfiguration;///< true if uplink timeAlignmentTimer value is based on mobility of UE
  bool m_isDl;///< indicates whether data transfer is in downlink or uplink direction
  Vector m_velocity;///< the velocity of the UE
  double m_interPacketInterval;///< the interval between packet transmission
  Time m_applicationStopTime; ///< the time at which the application stops sending data
  UeStatus m_status; ///< set of parameters to verify at end of simulation

  /// The current UE RRC state.
  LteUeRrc::State m_lastState;

  bool m_isUplinkOutOfSync; ///<true if UE loses its uplink synchronization at least once



}; // end of class LteUplinkSynchronizationTestCase

#endif /* LTE_TEST_UPLINK_SYNCHRONIZATION_H */
