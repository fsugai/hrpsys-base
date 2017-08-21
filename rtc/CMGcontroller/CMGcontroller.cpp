// -*- C++ -*-
/*!
 * @file  CMGcontroller.cpp * @brief  * $Date$ 
 *
 * $Id$ 
 */
#include "CMGcontroller.h"
#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>

typedef coil::Guard<coil::Mutex> Guard;


// Module specification
// <rtc-template block="module_spec">
static const char* cmgcontroller_spec[] =
  {
    "implementation_id", "CMGcontroller",
    "type_name",         "CMGcontroller",
    "description",       "CMG controller",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

CMGcontroller::CMGcontroller(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_qRefIn("qRef", m_qRef),
    m_rpyIn("rpy", m_rpy),
    m_baseRpyIn("baseRpyIn", m_baseRpy),
    m_qRefOut("q", m_qRef),
    m_CMGServicePort("CMGService"),
    cmg_mode(STOP),
    // </rtc-template>
    m_debugLevel(0)
{
  m_service0.cmg(this);
}

CMGcontroller::~CMGcontroller()
{
}


RTC::ReturnCode_t CMGcontroller::onInitialize()
{
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  bindParameter("debugLevel", m_debugLevel, "0");
  // Set InPort buffers
  addInPort("qRef", m_qRefIn);
  addInPort("rpy", m_rpyIn);
  addInPort("baseRpyIn", m_baseRpyIn);

  // Set OutPort buffer
  addOutPort("q", m_qRefOut);

  // Set service provider to Ports
  m_CMGServicePort.registerProvider("service0", "CMGService", m_service0);

  // Set service consumers to Ports

  // Set CORBA Service Ports
  addPort(m_CMGServicePort);

  // </rtc-template>
  RTC::Properties& prop = getProperties();
  coil::stringTo(m_dt, prop["dt"].c_str());

  m_robot = hrp::BodyPtr(new hrp::Body());

  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
      comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);

  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
			       CosNaming::NamingContext::_duplicate(naming.getRootContext())
                               )){
      std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
      return RTC::RTC_ERROR;
  }

  roll_joint_id = m_robot->link("CMG_JOINT0")->jointId;
  pitch_joint_id = m_robot->link("CMG_JOINT1")->jointId;
  spin_joint_id = m_robot->link("CMG_JOINT2")->jointId;
  std::cerr << "CMG roll  joint: " << roll_joint_id << std::endl;
  std::cerr << "CMG pitch joint: " << pitch_joint_id << std::endl;
  std::cerr << "CMG spin  joint: " << spin_joint_id << std::endl;

  m_qRef.data.length(m_robot->numJoints());
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t CMGcontroller::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t CMGcontroller::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t CMGcontroller::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

void CMGcontroller::startCMGcontroller()
{
    cmg_mode = START;
}

void CMGcontroller::stopCMGcontroller()
{
    cmg_mode = STOP;
}

RTC::ReturnCode_t CMGcontroller::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CMGcontroller::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CMGcontroller::onExecute(RTC::UniqueId ec_id)
{
    static int i=0;

    if (m_qRefIn.isNew()) {
        m_qRefIn.read();
    }
    if (m_baseRpyIn.isNew()) {
        m_baseRpyIn.read();
    }
    if (m_rpyIn.isNew()) {
        m_rpyIn.read();
    }
    Guard guard(m_mutex);

    m_robot->rootLink()->p = hrp::Vector3::Zero();
    m_robot->calcForwardKinematics();
    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    hrp::Matrix33 senR = sen->link->R * sen->localR;
    hrp::Matrix33 act_Rs(hrp::rotFromRpy(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y));    
    hrp::Matrix33 act_base =  act_Rs * (senR.transpose() * m_robot->rootLink()->R);
    hrp::Vector3 act_base_rpy = hrp::rpyFromRot(act_base);

    i++;

    const double cmg_J = 0.09;
    const double ref_spin_dq = 2 * M_PI * 500.0 / 60.0; // 5000 [rpm] -> [rad/s]
    const int init_time =2;
    const int acc_time = 10;
    const double cmg_th = 0.1;

    static double spin_q = 0.0;
    static double roll_q = 0.0;
    static double pitch_q = 0.0;

    double spin_dq = 0.0;
    double roll_dq = 0.0;
    double pitch_dq = 0.0;

    if(i > init_time/m_dt && i < acc_time/m_dt){
        spin_dq = ref_spin_dq * (i-init_time/m_dt) / (acc_time/m_dt-init_time/m_dt);
    }else if(i >= acc_time/m_dt){
        spin_dq = ref_spin_dq;
    }else{
        spin_dq = 0.0;
    }

    spin_q += spin_dq*m_dt;
    m_qRef.data[spin_joint_id] = spin_q;

    hrp::Vector3 tau_waist;
    hrp::Vector3 tau_cmg;
    hrp::Vector3 cmg_dq;
    hrp::Matrix33 cmg_tau_dq;
    hrp::Matrix33 R_cmg_waist;

    cmg_tau_dq << 0.0, 1/cmg_J/ref_spin_dq, 0.0,
        1/cmg_J/ref_spin_dq, 0.0, 0.0,
        0.0, 0.0, 0.0;

    R_cmg_waist = hrp::rotFromRpy(roll_q, pitch_q, 0.0);

    tau_waist = -hrp::Vector3::UnitZ().cross(act_base * hrp::Vector3::UnitZ());
    if(tau_waist.norm() > sin(cmg_th)){
        tau_waist = 40.0 * asin(tau_waist.norm()) * tau_waist.normalized();
        tau_cmg = R_cmg_waist * tau_waist;
        cmg_dq = cmg_tau_dq * tau_cmg;

        roll_dq = cmg_dq(0);
        pitch_dq = -cmg_dq(1);
    }else{
        if(roll_q < -0.01){
            roll_dq = 1.0;
        }else if(roll_q > 0.01){
            roll_dq = -1.0;
        }
        if(pitch_q < -0.01){
            pitch_dq = 1.0;
        }else if(pitch_q > 0.01){
            pitch_dq = -1.0;
        }
    }


    if(i > init_time/m_dt && cmg_mode == START){
        roll_q += roll_dq*m_dt;
        if(roll_q>1.4)
            roll_q = 1.4;
        else if(roll_q < -1.4)
            roll_q = -1.4;
        m_qRef.data[roll_joint_id] = roll_q;

        pitch_q += pitch_dq*m_dt;
        if(pitch_q>1.4)
            pitch_q = 1.4;
        else if(pitch_q < -1.4)
            pitch_q = -1.4;
        m_qRef.data[pitch_joint_id] = pitch_q;
    }

    m_qRefOut.write();
 
   return RTC::RTC_OK;
}
/*
RTC::ReturnCode_t CMGcontroller::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t CMGcontroller::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t CMGcontroller::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t CMGcontroller::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t CMGcontroller::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

extern "C"
{
 
  void CMGcontrollerInit(RTC::Manager* manager)
  {
    coil::Properties profile(cmgcontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<CMGcontroller>,
                             RTC::Delete<CMGcontroller>);
  }
  
};



