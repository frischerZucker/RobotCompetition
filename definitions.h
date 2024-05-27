/* 
 * File:   definitions.h
 * Author: moritz
 *
 * Created on May 27, 2024, 3:42 PM
 */

#ifndef DEFINITIONS_H
#define	DEFINITIONS_H

#ifdef	__cplusplus
extern "C" {
#endif

#define FORWARD 1
#define BACKWARD 0
    
    /*
     * R, L: 0...255
     */
    void gaspedal(char R, char L);
    /*
     * 1: vorwärts, 0: rückwärts
     */
    void gangschaltung(char R, char L);
    
    void rotate_to_led();


#ifdef	__cplusplus
}
#endif

#endif	/* DEFINITIONS_H */

