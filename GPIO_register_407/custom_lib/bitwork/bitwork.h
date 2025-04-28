/*
 * bitwork.h
 *
 *  Created on: Apr 28, 2025
 *      Author: User
 */

#ifndef BITWORK_BITWORK_H_
#define BITWORK_BITWORK_H_


#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#endif /* BITWORK_BITWORK_H_ */
