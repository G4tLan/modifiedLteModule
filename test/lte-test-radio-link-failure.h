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

#ifndef LTE_TEST_RADIO_LINK_FAILURE_H
#define LTE_TEST_RADIO_LINK_FAILURE_H


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
 * \sa ns3::LteRadioLinkFailureTestCase
 */
class LteRadioLinkFailureTestSuite : public TestSuite
{
public:
  LteRadioLinkFailureTestSuite ();
};

/**
 * \ingroup lte
 *
 * \brief Testing the cell reselection procedure by UE at IDLE state
 */
class LteRadioLinkFailureTestCase : public TestCase
{
public:

  /**
   * \brief Creates an instance of the radio link failure test case.
   * \param name name of this test
   * \param nEnbs number of eNodeBs
   * \param nUes number of UEs
   * \param isIdealRrc if true, simulation uses Ideal RRC protocol, otherwise
   *                   simulation uses Real RRC protocol
   * \param uePositionList Position of the UEs
   * \param enbPositionList Position of the eNodeBs
   * \param checkConnectedList the time at which UEs should have an active RRC connection
   */
  LteRadioLinkFailureTestCase (std::string name, uint32_t nEnbs, uint32_t nUes, bool isIdealRrc,
                               std::vector<Vector> uePositionList, std::vector<Vector> enbPositionList,
                               std::vector<Time> checkConnectedList);

  virtual ~LteRadioLinkFailureTestCase ();

private:
  /**
   * \brief Setup the simulation according to the configuration set by the
   *        class constructor, run it, and verify the result.
   */
  virtual void DoRun ();

  /**
   * Check connected function
   * \param ueDevice the UE device
   * \param enbDevices the ENB devices
   */
  void CheckConnected (Ptr<NetDevice> ueDevice, NetDeviceContainer enbDevices);

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
   * \brief This callback function is executed when UE RRC receives an in-sync or out-of-sync indication
   * \param context the context string
   * \param imsi the IMSI
   * \param rnti the RNTI
   * \param cellId the cell ID
   * \param type in-sync or out-of-sync indication
   * \param count the number of in-sync or out-of-sync indications
   */
  void PhySyncDetectionCallback(std::string context, uint64_t imsi, uint16_t rnti,
                        uint16_t cellId, std::string type, uint16_t count);

  /**
   * \brief This callback function is executed when radio link failure is detected
   * \param context the context string
   * \param imsi the IMSI
   * \param rnti the RNTI
   * \param cellId the cell ID
   */
  void RadioLinkFailureCallback (std::string context, uint64_t imsi, uint16_t rnti, uint16_t cellId);

  uint32_t m_nEnbs;///<number of eNodeBs
  uint32_t m_nUes; ///<number of UEs
  bool m_isIdealRrc;///< whether the LTE is configured to use ideal RRC
  std::vector<Vector> m_uePositionList;///<Position of the UEs
  std::vector<Vector> m_enbPositionList;///<Position of the eNodeBs
  std::vector<Time> m_checkConnectedList;///<the time at which UEs should have an active RRC connection

  /// The current UE RRC state.
  LteUeRrc::State m_lastState;

  bool m_radioLinkFailureDetected;  ///< true if radio link fails
  uint32_t m_numOfInSyncIndications; ///< number of in-sync indications detected
  uint32_t m_numOfOutOfSyncIndications; ///< ///< number of out-of-sync indications detected

}; // end of class LteRadioLinkFailureTestCase

#endif /* LTE_TEST_RADIO_LINK_FAILURE_H */
