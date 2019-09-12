/* starting port:

- post first on lap
- floats in and out ???
- output list of coeffs...

 */

/*  Linear Predictive Coding - PARCOR and residual generation
 *  Copyright (C) 2005 Nicolas Chetry <okin@altern.org>
 *  and Edward Kelly <morph_2016@yahoo.co.uk>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

//#include "m_pd.h"
#include <math.h>
#define MAXPOLES 100

//static t_class *lpc_tilde_class;

typedef struct _lpc_schur
{
  float *c_input, *c_residual; // in is input and residual is output - but we need memory for these... max size of chunk
  /* just so we don't get clicks */
  float x_last_parcors[MAXPOLES];
  float x_parcors[MAXPOLES];
} t_lpc_schur;

typedef struct _lpc_tilde
{
  float x_order, x_lastorder;
  //  float *parcor_list;
  t_lpc_schur x_schur;
} t_lpc_tilde;

void lpc_tilde_perform() // the action!
{
  //    t_lpc_tilde     *x =     (t_lpc_tilde *)(w[1]);
  //    t_lpc_schur *schur =     (t_lpc_schur *)(w[2]);
  //    uint16_t              n =               (int)(w[3]);

  //  we need x, n (blocksize?) and schur object
  
    float        *in = schur->c_input;
    float       *res = schur->c_residual;
//  t_float   *parcors = schur->PARCORS;
//  float       *acf = schur->ACF;
//  float         *k = schur->K;
//  float         *p = schur->P;
//  float         *r = schur->r;
  uint16_t          ord = (int)x->x_order;
  uint16_t        l_ord = (int)x->x_lastorder;
//  float parcors[ord];
  float acf[ord+1];
  float k[ord+1];
  float p[ord+1];
  float r[ord+1];
  float mem[ord];
  uint16_t i, j, bid, y, z;
  uint16_t g, h;
  float tmp, temp, sav, di;
  float parcor_1;  
  for (i=0; i<ord; i++) 
  {
    //    SETFLOAT (&schur->x_parcors[i],0);
    schur->x_parcors[i]=0.0f;
    mem[i] = 0.0;
  }
  for (j=0; j<=ord; j++) 
  {
    acf[j] = 0;
    for (i=j; i<n; i++) 
	{
	  acf[j] += in[i]*in[i-j];
	}
  }
  if (acf[0] == 0) 
  {
    for (i=0; i<ord; i++)
	{
	  //    SETFLOAT (&schur->x_parcors[i],0);
	  schur->x_parcors[i]=0.0f;
	}
  }
  for (i=0; i<=ord; i++)
  {
    p[i]=acf[i];
	if (i > 0 && i < ord)
	{
	  k[ord+1-i] = acf[i];
	}
  }
  /* Schurr recursion */
  for (y=1; y<=ord; y++)
  {
    if (p[0] < fabs (p[1]))
	{
	  for (i=y; i<=ord; i++)
	  {
	    r[i] = 0;
	  }
	  for (bid=1; bid <=ord; bid++)
	  {
	    //            SETFLOAT (&schur->x_parcors[bid-1],r[bid]);
	    schur->x_parcors[bid-1] = r[bid];
	  }
	}
	r[y] = fabs(p[1])/p[0];
	
	if (p[1] >0)
	{
	  r[y] = -1.*r[y];
	}
	if (y==ord)
	{
	  for (bid=1; bid <=ord; bid++)
	  {
	    //            SETFLOAT (&schur->x_parcors[bid-1],r[bid]);
	    schur->x_parcors[bid-1] = r[bid];
	  }
	}
	p[0] += p[1]*r[y];
	
	for (z=1; z <=ord-y; z++)
	{
	  p[z]        = p[z+1] + k[ord+1-z]*r[y];
	  k[ord+1-z] += p[z+1] * r[y];
	}
  }
  for (bid=1; bid <=ord; bid++)
  {
    //    SETFLOAT (&schur->x_parcors[bid-1],r[bid]);
	    schur->x_parcors[bid-1] = r[bid];
      }

  //  parcor_1 = atom_getfloatarg(0,ord,schur->x_parcors); /* in order to avoid nil coefficients */
    parcor_1 = schur->x_parcors[0]; /* in order to avoid nil coefficients */

  if (parcor_1 > 1e-5 || parcor_1 < -1e-5)
    {
      //    outlet_list(x->parcor_list,gensym("list"),ord,schur->x_parcors); // ???

  /* Analysis FIR lattice filtering */
      for (g=0; g<n; g++)
	{
        
    /* Analysis - Lattice structure */
	  sav = di = in[g];
	  for (i=0; i<ord; i++)
	    {
	      //	      float parcor = atom_getfloatarg (i,ord,schur->x_parcors);
	      float parcor = schur->x_parcors[i];
	      //	      SETFLOAT (&schur->x_last_parcors[i],parcor);
	      schur->x_last_parcors[i] = parcor;

	      x->x_lastorder = ord;
	      temp = mem[i] + parcor*di;
	      di += parcor*mem[i];
	      mem[i] = sav;
	      sav = temp;
	    }
	  res[g] = di;
            
	  }  /* next g */
    }
  else 
    {
      //      outlet_list(x->parcor_list,gensym("list"),l_ord,schur->x_last_parcors);
      for (g=0; g<n; g++)
	{
	  res[g] = 0;
	}
    }
  //  return(w+4); //??
}

/*
void *lpc_tilde_dsp(t_lpc_tilde *x, t_signal **sp)
{
  x->x_schur.c_input = sp[0]->s_vec;
  x->x_schur.c_residual = sp[1]->s_vec;
  dsp_add(lpc_tilde_perform, 3, x, &x->x_schur, sp[0]->s_n);
  return (void *)x;
}
*/

 /*
void *lpc_tilde_new(float f)
{
  t_lpc_tilde *x = (t_lpc_tilde *)pd_new(lpc_tilde_class);
  x->x_order = f >= 1 ? (uint16_t)f : 5;
  
  floatinlet_new(&x->x_obj,&x->x_order);
  outlet_new(&x->x_obj, &s_signal);
  x->parcor_list = outlet_new(&x->x_obj, &s_list);
  return (void *)x;
}
 */

// what do we need to initialise

// order (32), blocksize (1024) (in pd 2 overlapping windows)

// in signal-> hip~1000 -> lpc~ // with excitation? -> lpcreson~ -> lop~ 1000 -> hanning -> out
