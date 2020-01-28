/*
 * Generated by asn1c-0.9.24 (http://lionet.info/asn1c)
 * From ASN.1 module "DENM-PDU-Descriptions"
 * 	found in "its_facilities_pdu_all.asn"
 * 	`asn1c -fnative-types -gen-PER`
 */

#include "ImpactReductionContainer.h"

static asn_TYPE_member_t asn_MBR_ImpactReductionContainer_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, heightLonCarrLeft),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_HeightLonCarr,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"heightLonCarrLeft"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, heightLonCarrRight),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_HeightLonCarr,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"heightLonCarrRight"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, posLonCarrLeft),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PosLonCarr,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"posLonCarrLeft"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, posLonCarrRight),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PosLonCarr,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"posLonCarrRight"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, positionOfPillars),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PositionOfPillars,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"positionOfPillars"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, posCentMass),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PosCentMass,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"posCentMass"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, wheelBaseVehicle),
		(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_WheelBaseVehicle,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"wheelBaseVehicle"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, turningRadius),
		(ASN_TAG_CLASS_CONTEXT | (7 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_TurningRadius,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"turningRadius"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, posFrontAx),
		(ASN_TAG_CLASS_CONTEXT | (8 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PosFrontAx,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"posFrontAx"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, positionOfOccupants),
		(ASN_TAG_CLASS_CONTEXT | (9 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PositionOfOccupants,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"positionOfOccupants"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, vehicleMass),
		(ASN_TAG_CLASS_CONTEXT | (10 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleMass,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"vehicleMass"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ImpactReductionContainer, requestResponseIndication),
		(ASN_TAG_CLASS_CONTEXT | (11 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RequestResponseIndication,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"requestResponseIndication"
		},
};
static ber_tlv_tag_t asn_DEF_ImpactReductionContainer_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_TYPE_tag2member_t asn_MAP_ImpactReductionContainer_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* heightLonCarrLeft at 66 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* heightLonCarrRight at 67 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* posLonCarrLeft at 68 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* posLonCarrRight at 69 */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* positionOfPillars at 70 */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 }, /* posCentMass at 71 */
    { (ASN_TAG_CLASS_CONTEXT | (6 << 2)), 6, 0, 0 }, /* wheelBaseVehicle at 72 */
    { (ASN_TAG_CLASS_CONTEXT | (7 << 2)), 7, 0, 0 }, /* turningRadius at 73 */
    { (ASN_TAG_CLASS_CONTEXT | (8 << 2)), 8, 0, 0 }, /* posFrontAx at 74 */
    { (ASN_TAG_CLASS_CONTEXT | (9 << 2)), 9, 0, 0 }, /* positionOfOccupants at 75 */
    { (ASN_TAG_CLASS_CONTEXT | (10 << 2)), 10, 0, 0 }, /* vehicleMass at 76 */
    { (ASN_TAG_CLASS_CONTEXT | (11 << 2)), 11, 0, 0 } /* requestResponseIndication at 78 */
};
static asn_SEQUENCE_specifics_t asn_SPC_ImpactReductionContainer_specs_1 = {
	sizeof(struct ImpactReductionContainer),
	offsetof(struct ImpactReductionContainer, _asn_ctx),
	asn_MAP_ImpactReductionContainer_tag2el_1,
	12,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* Start extensions */
	-1	/* Stop extensions */
};
asn_TYPE_descriptor_t asn_DEF_ImpactReductionContainer = {
	"ImpactReductionContainer",
	"ImpactReductionContainer",
	SEQUENCE_free,
	SEQUENCE_print,
	SEQUENCE_constraint,
	SEQUENCE_decode_ber,
	SEQUENCE_encode_der,
	SEQUENCE_decode_xer,
	SEQUENCE_encode_xer,
	SEQUENCE_decode_uper,
	SEQUENCE_encode_uper,
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_ImpactReductionContainer_tags_1,
	sizeof(asn_DEF_ImpactReductionContainer_tags_1)
		/sizeof(asn_DEF_ImpactReductionContainer_tags_1[0]), /* 1 */
	asn_DEF_ImpactReductionContainer_tags_1,	/* Same as above */
	sizeof(asn_DEF_ImpactReductionContainer_tags_1)
		/sizeof(asn_DEF_ImpactReductionContainer_tags_1[0]), /* 1 */
	0,	/* No PER visible constraints */
	asn_MBR_ImpactReductionContainer_1,
	12,	/* Elements count */
	&asn_SPC_ImpactReductionContainer_specs_1	/* Additional specs */
};

