/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "CAM-PDU-Descriptions"
 * 	found in "its_facilities_pdu_all.asn"
 * 	`asn1c -fnative-types -gen-PER -pdu=auto`
 */

#ifndef	_SafetyCarContainer_H_
#define	_SafetyCarContainer_H_


#include <asn_application.h>

/* Including external dependencies */
#include "LightBarSirenInUse.h"
#include "TrafficRule.h"
#include "SpeedLimit.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct CauseCode;

/* SafetyCarContainer */
typedef struct SafetyCarContainer {
	LightBarSirenInUse_t	 lightBarSirenInUse;
	struct CauseCode	*incidentIndication	/* OPTIONAL */;
	TrafficRule_t	*trafficRule	/* OPTIONAL */;
	SpeedLimit_t	*speedLimit	/* OPTIONAL */;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SafetyCarContainer_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SafetyCarContainer;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "CauseCode.h"

#endif	/* _SafetyCarContainer_H_ */
#include <asn_internal.h>
