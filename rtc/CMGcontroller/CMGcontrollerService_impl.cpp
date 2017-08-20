// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "CMGcontrollerService_impl.h"

CMGcontrollerService_impl::CMGcontrollerService_impl()
{
}

CMGcontrollerService_impl::~CMGcontrollerService_impl()
{
}

void CMGcontrollerService_impl::echo(const char *msg)
{
	std::cout << "CMGcontrollerService: " << msg << std::endl;
}

void CMGcontrollerService_impl::cmg(CMGcontroller *i_cmg)
{
	m_cmg = i_cmg;
}

