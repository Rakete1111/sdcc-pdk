/* pdkadr.c */

/*
 *  Copyright (C) 1998-2009  Alan R. Baldwin
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Alan R. Baldwin
 * 721 Berkeley St.
 * Kent, Ohio  44240
 *
 *      This Assember Ported by
 *      John L. Hartman (JLH)
 *      jhartman at compuserve dot com
 *      noice at noicedebugger dot com
 *
 */

#include "asxxxx.h"
#include "pdk.h"


/*  Classify argument as to address mode */
int
addr(struct expr *esp)
{
        int c = getnb();

        if (c == '#') {
                /* Immediate mode */
                expr(esp, 0);
                esp->e_mode = S_K;
        } else
        if (c == 'o' && ((c = getnb()) == 'v' || (unget(c), 0))) {
                /* OV bit of ACC flag */
                esp->e_mode = S_K;
                esp->e_addr = 3;
        } else
        if (c == 'a' && ((c = getnb()) == 'c' || (unget(c), 0))) {
                /* AC bit of ACC flag */
                esp->e_mode = S_K;
                esp->e_addr = 2;
        } else
        if (c == 's' && ((c = getnb()) == 'p' || (unget(c), 0))) {
                /* Stack */
                esp->e_mode = S_IO;
                esp->e_addr = 2;
        } else
        if (c == 'a') {
                /* Accumulator */
                esp->e_mode = S_A;
        } else
        if (c == 'c') {
                /* C bit of ACC flag */
                esp->e_mode = S_K;
                esp->e_addr = 1;
        } else
        if (c == 'z') {
                /* Z bit of ACC flag */
                esp->e_mode = S_K;
                esp->e_addr = 0;
        }
        else {
                unget(c);
                /* Memory address */
                expr(esp, 0);
                esp->e_mode = S_M;
        }

        return (esp->e_mode);
}
