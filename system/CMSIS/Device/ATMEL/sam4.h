/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2014, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following condition is met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */
#ifndef _SAM4_INCLUDED_
#define _SAM4_INCLUDED_

#if defined (__SAM4C8C_0__)
#include "sam4c/include/sam4c.h"
#elif defined (__SAM4C8C_1__)
#include "sam4c/include/sam4c.h"
#elif defined (__SAM4C16C_0__)
#include "sam4c/include/sam4c.h"
#elif defined (__SAM4C16C_1__)
#include "sam4c/include/sam4c.h"
#elif defined (__SAM4CMP8C_0__)
#include "sam4cm/include/sam4cm.h"
#elif defined (__SAM4CMP8C_1__)
#include "sam4cm/include/sam4cm.h"
#elif defined (__SAM4CMP16C_0__)
#include "sam4cm/include/sam4cm.h"
#elif defined (__SAM4CMP16C_1__)
#include "sam4cm/include/sam4cm.h"
#elif defined (__SAM4CMS8C_0__)
#include "sam4cm/include/sam4cm.h"
#elif defined (__SAM4CMS8C_1__)
#include "sam4cm/include/sam4cm.h"
#elif defined (__SAM4CMS16C_0__)
#include "sam4cm/include/sam4cm.h"
#elif defined (__SAM4CMS16C_1__)
#include "sam4cm/include/sam4cm.h"
#elif defined (__SAM4CP16B_0__)
#include "sam4cp/include/sam4cp.h"
#elif defined (__SAM4CP16B_1__)
#include "sam4cp/include/sam4cp.h"
#elif defined (__SAM4CP16C_0__)
#include "sam4cp/include/sam4cp.h"
#elif defined (__SAM4CP16C_1__)
#include "sam4cp/include/sam4cp.h"
#elif defined (__SAM4C32C_0__)
#include "sam4c32/include/sam4c32.h"
#elif defined (__SAM4C32C_1__)
#include "sam4c32/include/sam4c32.h"
#elif defined (__SAM4C32E_0__)
#include "sam4c32/include/sam4c32.h"
#elif defined (__SAM4C32E_1__)
#include "sam4c32/include/sam4c32.h"
#elif defined (__SAM4CMP32C_0__)
#include "sam4cm32/include/sam4cm32.h"
#elif defined (__SAM4CMP32C_1__)
#include "sam4cm32/include/sam4cm32.h"
#elif defined (__SAM4CMS32C_0__)
#include "sam4cm32/include/sam4cm32.h"
#elif defined (__SAM4CMS32C_1__)
#include "sam4cm32/include/sam4cm32.h"
#elif defined (__ATSAM4LC2A__) || defined (__SAM4LC2A__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LC2B__) || defined (__SAM4LC2B__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LC2C__) || defined (__SAM4LC2C__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LC4A__) || defined (__SAM4LC4A__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LC4B__) || defined (__SAM4LC4B__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LC4C__) || defined (__SAM4LC4C__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LC8A__) || defined (__SAM4LC8A__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LC8B__) || defined (__SAM4LC8B__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LC8C__) || defined (__SAM4LC8C__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LS2A__) || defined (__SAM4LS2A__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LS2B__) || defined (__SAM4LS2B__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LS2C__) || defined (__SAM4LS2C__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LS4A__) || defined (__SAM4LS4A__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LS4B__) || defined (__SAM4LS4B__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LS4C__) || defined (__SAM4LS4C__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LS8A__) || defined (__SAM4LS8A__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LS8B__) || defined (__SAM4LS8B__)
#include "sam4l/include/sam4l.h"
#elif defined (__ATSAM4LS8C__) || defined (__SAM4LS8C__)
#include "sam4l/include/sam4l.h"
#elif defined (__SAM4S16C__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4S16B__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4S2A__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4S2B__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4S2C__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4S4A__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4S4B__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4S4C__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4S8C__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4S8B__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4SA16B__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4SA16C__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4SD16B__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4SD16C__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4SD32B__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4SD32C__)
#include "sam4s/include/sam4s.h"
#elif defined (__SAM4N8A__)
#include "sam4n/include/sam4n.h"
#elif defined (__SAM4N8B__)
#include "sam4n/include/sam4n.h"
#elif defined (__SAM4N8C__)
#include "sam4n/include/sam4n.h"
#elif defined (__SAM4N16B__)
#include "sam4n/include/sam4n.h"
#elif defined (__SAM4N16C__)
#include "sam4n/include/sam4n.h"
#elif defined (__SAM4E8E__)
#include "sam4e/include/sam4e.h"
#elif defined (__SAM4E16E__)
#include "sam4e/include/sam4e.h"
#elif defined (__SAM4E8C__)
#include "sam4e/include/sam4e.h"
#elif defined (__SAM4E16C__)
#include "sam4e/include/sam4e.h"
#elif defined (__SAM4SP32A__)
#include "sam4sp/include/sam4sp.h"
#endif

#endif /* _SAM4_INCLUDED_ */
