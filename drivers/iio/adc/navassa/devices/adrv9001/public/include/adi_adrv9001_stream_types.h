/**
 * \file
 * \brief Contains ADRV9001 API Stream data types
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_STREAM_TYPES_H_
#define _ADI_ADRV9001_STREAM_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

//#define ADRV9001_IMAGEINFOSIZE		4
#define ADRV9001_MAINSTREAMSIZEMAX	   16*1024	/* Memory size for Main stream processor */
#define ADRV9001_TXSTREAMSIZEMAX		4*1024	/* Memory size for Tx stream processor */
#define ADRV9001_RXSTREAMSIZEMAX		4*1024	/* Memory size for Rx stream processor */
//#define ADRV9001_ORXSTREAMSIZEMAX		4*1024	/* Memory size for ORx stream processor */
#define ADRV9001_MAINNUM				1		/* Maximum number of main stream processors */
#define ADRV9001_TXNUM					2		/* Maximum number of Tx channels */
#define ADRV9001_RXNUM					2		/* Maximum number of Rx channels */
//#define ADRV9001_ORXNUM				2		/* Maximum number of ORx channels */
#define ADRV9001_MAX_NUM_STREAM         5	    /* Maximum number of stream processors */

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_STREAM_TYPES_H_ */
