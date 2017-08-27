// -*- C++ -*-
/*!
 * @file  CMGcontroller.h * @brief  * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef CMGCONTROLLER_H
#define CMGCONTROLLER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/EigenTypes.h>

#include "CMGcontrollerService_impl.h"

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class CMGcontroller  : public RTC::DataFlowComponentBase
{
 public:
  CMGcontroller(RTC::Manager* manager);
  ~CMGcontroller();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry() 
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);

  void startCMGcontroller();
  void stopCMGcontroller();
  void getParameter(OpenHRP::CMGcontrollerService::cmgParam& i_cmgp);
  void setParameter(const OpenHRP::CMGcontrollerService::cmgParam& i_cmgp);

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  InPort<TimedDoubleSeq> m_qRefIn;
  TimedOrientation3D m_rpy;
  TimedOrientation3D m_baseRpy;
  TimedPoint3D m_zmpRef;
  TimedPoint3D m_zmp;
  InPort<TimedOrientation3D> m_rpyIn;
  InPort<TimedOrientation3D> m_baseRpyIn;
  InPort<TimedPoint3D> m_zmpRefIn;
  InPort<TimedPoint3D> m_zmpIn;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_qRef;
  OutPort<TimedDoubleSeq> m_qRefOut;

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  RTC::CorbaPort m_CMGServicePort;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  CMGcontrollerService_impl m_service0;

  // </rtc-template>

 private:
  hrp::BodyPtr m_robot;
  double m_dt;
  hrp::Matrix33 input_baseRot;
  unsigned int m_debugLevel;
  coil::Mutex m_mutex;
  int roll_joint_id;
  int pitch_joint_id;
  int spin_joint_id;
  enum c_mode {STOP, START} cmg_mode;

  bool is_sim;
  double spin_rpm;
  double deadband_th;
  double back_dq;
  double kp;
};


extern "C"
{
  DLL_EXPORT void CMGcontrollerInit(RTC::Manager* manager);
};

#endif // CMGCONTROLLER_H

