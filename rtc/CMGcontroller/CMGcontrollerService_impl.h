// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __CMG_CONTROLLER_SERVICE_H__
#define __CMG_CONTROLLER_SERVICE_H__

#include "hrpsys/idl/CMGcontrollerService.hh"

class CMGcontroller;

class CMGcontrollerService_impl
	: public virtual POA_OpenHRP::CMGcontrollerService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	CMGcontrollerService_impl();

	/**
	   \brief destructor
	*/
	virtual ~CMGcontrollerService_impl();

	void echo(const char *msg);
	void cmg(CMGcontroller *i_cmg);
private:
	CMGcontroller *m_cmg;
};

#endif
