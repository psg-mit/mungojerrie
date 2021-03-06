/**
  @file

  @ingroup cudd

  @brief Functions that translate BDDs to ZDDs.

  @author Hyong-kyoon Shin, In-Ho Moon

  @copyright@parblock
  Copyright (c) 1995-2021, Regents of the University of Colorado

  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

  Neither the name of the University of Colorado nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
  @endparblock

*/

#include "util.h"
#include "cuddInt.h"

/*---------------------------------------------------------------------------*/
/* Constant declarations                                                     */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/* Stucture declarations                                                     */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/* Type declarations                                                         */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/* Variable declarations                                                     */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/* Macro declarations                                                        */
/*---------------------------------------------------------------------------*/

/** \cond */

/*---------------------------------------------------------------------------*/
/* Static function prototypes                                                */
/*---------------------------------------------------------------------------*/

static DdNode * zddPortFromBddStep (DdManager *dd, DdNode *B, int expected);
static DdNode * zddPortFromBddNegCofStep(DdManager *dd, DdNode *B, DdNode *cube, int *indices, int expected);
static DdNode * zddPortToBddStep (DdManager *dd, DdNode *f, int depth);
static DdNode * zddPortToBddNegCofStep (DdManager *dd, DdNode *f, DdNode *cube, int *indices, int depth);

/** \endcond */


/*---------------------------------------------------------------------------*/
/* Definition of exported functions                                          */
/*---------------------------------------------------------------------------*/


/**
  @brief Converts a %BDD into a %ZDD.

  @details This function assumes that there is a one-to-one
  correspondence between the %BDD variables and the %ZDD variables, and
  that the variable order is the same for both types of
  variables. These conditions are established if the %ZDD variables are
  created by one call to Cudd_zddVarsFromBddVars with multiplicity = 1.

  @return a pointer to the resulting %ZDD if successful; NULL otherwise.

  @sideeffect None

  @see Cudd_zddPortToBdd Cudd_zddPortFromBddNegCof Cudd_zddVarsFromBddVars

*/
DdNode *
Cudd_zddPortFromBdd(
  DdManager * dd /**< manager */,
  DdNode * B /**< %BDD to be converted */)
{
    DdNode *res;

    do {
	dd->reordered = 0;
	res = zddPortFromBddStep(dd,B,0);
    } while (dd->reordered == 1);
    if (dd->errorCode == CUDD_TIMEOUT_EXPIRED && dd->timeoutHandler) {
        dd->timeoutHandler(dd, dd->tohArg);
    }

    return(res);

} /* end of Cudd_zddPortFromBdd */


/**
  @brief Converst a negative cofactor of a %BDD into a %ZDD.

  @details Cube is a conjunction of positive literal.  The function for which
  a %ZDD is built is the negative cofactor of B w.r.t. all the variables that
  appear in cube.  This function assumes that there is a one-to-one
  correspondence between the %BDD variables and the %ZDD variables, and
  that the variable order is the same for both types of variables.
  These conditions are established if the %ZDD variables are created by
  one call to Cudd_zddVarsFromBddVars() with multiplicity = 1.

  @return a pointer to the resulting %ZDD if successful; NULL otherwise.

  @sideeffect None

  @see Cudd_zddPortFromBdd Cudd_zddPortToBddNegCof Cudd_zddVarsFromBddVars
*/
DdNode *
Cudd_zddPortFromBddNegCof(
  DdManager *dd /**< manager */,
  DdNode *B /**< %BDD to be converted */,
  DdNode *cube /**< %BDD variables to be cofactored */)
{
    size_t i;
    int *indices;
    DdNode *res, *scan;
    DdNode *lzero = Cudd_Not(DD_ONE(dd));
    size_t size = dd->size;

    if ((size_t) dd->sizeZ != size) {
        /* There should be a 1-to-1 correspondence between BDD variables
        ** and ZDD variables. */
        dd->errorCode = CUDD_WRONG_PRECONDITIONS;
        return(NULL);
    }

    /* Create look up table for variables to be cofactored. */
    indices = ALLOC(int, size);
    if (indices == NULL) {
        dd->errorCode = CUDD_MEMORY_OUT;
        return(NULL);
    }
    for (i = 0; i < size; i++) {
        indices[i] = 0;
    }
    scan = cube;
    while (scan != DD_ONE(dd)) {
        if (Cudd_IsComplement(scan) || cuddE(scan) != lzero) {
            /* Argument cube is not a positive cube. */
            dd->errorCode = CUDD_INVALID_ARG;
            FREE(indices);
            return(NULL);
        }
        indices[scan->index] = 1;
        scan = cuddT(scan);
    }

    do {
        dd->reordered = 0;
        res = zddPortFromBddNegCofStep(dd, B, cube, indices, 0);
    } while (dd->reordered == 1);
    FREE(indices);
    if (dd->errorCode == CUDD_TIMEOUT_EXPIRED && dd->timeoutHandler) {
        dd->timeoutHandler(dd, dd->tohArg);
    }

    return(res);
    
} /* end of Cudd_zddPortFromBddNegCof */


/**
  @brief Converts a %ZDD into a %BDD.

  @return a pointer to the resulting %BDD if successful; NULL otherwise.

  @sideeffect None

  @see Cudd_zddPortFromBdd

*/
DdNode *
Cudd_zddPortToBdd(
  DdManager * dd,
  DdNode * f)
{
    DdNode *res;

    do {
	dd->reordered = 0;
	res = zddPortToBddStep(dd,f,0);
    } while (dd->reordered == 1);
    if (dd->errorCode == CUDD_TIMEOUT_EXPIRED && dd->timeoutHandler) {
        dd->timeoutHandler(dd, dd->tohArg);
    }

    return(res);

} /* end of Cudd_zddPortToBdd */


/**
  @brief Converts the negative cofactor of a %ZDD into a %BDD.

  @details Cube is a conjunction of positive literal.  The function for which
  a %BDD is built is the negative cofactor of f w.r.t. all the variables that
  appear in cube.  This function assumes that there is a one-to-one
  correspondence between the %BDD variables and the %ZDD variables, and
  that the variable order is the same for both types of variables.
  These conditions are established if the %ZDD variables are created by
  one call to Cudd_zddVarsFromBddVars() with multiplicity = 1.

  @return a pointer to the resulting %BDD if successful; NULL otherwise.

  @sideeffect None

  @see Cudd_zddPortToBdd Cudd_zddPortFromBddNegCof

*/
DdNode *
Cudd_zddPortToBddNegCof(
  DdManager * dd /**< manager */,
  DdNode * f /**< %ZDD to be converted */,
  DdNode *cube /**< %BDD variables to be cofactored */)
{
    size_t i;
    int *indices;
    DdNode *res, *scan;
    DdNode *lzero = Cudd_Not(DD_ONE(dd));
    size_t size = dd->size;

    if ((size_t) dd->sizeZ != size) {
        /* There should be a 1-to-1 correspondence between BDD variables
        ** and ZDD variables. */
        dd->errorCode = CUDD_WRONG_PRECONDITIONS;
        return(NULL);
    }

    /* Create look up table for variables to be cofactored. */
    indices = ALLOC(int, size);
    if (indices == NULL) {
        dd->errorCode = CUDD_MEMORY_OUT;
        return(NULL);
    }
    for (i = 0; i < size; i++) {
        indices[i] = 0;
    }
    scan = cube;
    while (scan != DD_ONE(dd)) {
        if (Cudd_IsComplement(scan) || cuddE(scan) != lzero) {
            /* Argument cube is not a positive cube. */
            dd->errorCode = CUDD_INVALID_ARG;
            FREE(indices);
            return(NULL);
        }
        indices[scan->index] = 1;
        scan = cuddT(scan);
    }

    do {
	dd->reordered = 0;
	res = zddPortToBddNegCofStep(dd, f, cube, indices, 0);
    } while (dd->reordered == 1);
    FREE(indices);
    if (dd->errorCode == CUDD_TIMEOUT_EXPIRED && dd->timeoutHandler) {
        dd->timeoutHandler(dd, dd->tohArg);
    }

    return(res);

} /* end of Cudd_zddPortToBddNegCof */


/*---------------------------------------------------------------------------*/
/* Definition of internal functions                                          */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Definition of static functions                                            */
/*---------------------------------------------------------------------------*/


/**
  @brief Performs the recursive step of Cudd_zddPortFromBdd().

  @sideeffect None

*/
static DdNode *
zddPortFromBddStep(
  DdManager * dd /**< manager */,
  DdNode * B /**< %BDD to be converted */,
  int expected /**< expected recursion depth */)
{
    DdNode	*res, *prevZdd, *t, *e;
    DdNode	*Breg, *Bt, *Be;
    int		id;
    int		level;

    statLine(dd);
    /* Terminal cases. */
    if (B == Cudd_Not(DD_ONE(dd)))
	return(DD_ZERO(dd));
    if (B == DD_ONE(dd)) {
	if (expected >= dd->sizeZ) {
	    return(DD_ONE(dd));
	} else {
	    return(dd->univ[expected]);
	}
    }

    Breg = Cudd_Regular(B);

    /* Computed table look-up. */
    res = cuddCacheLookup1Zdd(dd,Cudd_zddPortFromBdd,B);
    if (res != NULL) {
	level = cuddI(dd,Breg->index);
	/* Adding DC vars. */
	if (expected < level) {
	    /* Add suppressed variables. */
	    cuddRef(res);
	    for (level--; level >= expected; level--) {
		prevZdd = res;
		id = dd->invperm[level];
		res = cuddZddGetNode(dd, id, prevZdd, prevZdd);
		if (res == NULL) {
		    Cudd_RecursiveDerefZdd(dd, prevZdd);
		    return(NULL);
		}
		cuddRef(res);
		Cudd_RecursiveDerefZdd(dd, prevZdd);
	    }
	    cuddDeref(res);
	}
	return(res);
    }	/* end of cache look-up */

    if (Cudd_IsComplement(B)) {
	Bt = Cudd_Not(cuddT(Breg));
	Be = Cudd_Not(cuddE(Breg));
    } else {
	Bt = cuddT(Breg);
	Be = cuddE(Breg);
    }

    id = (int) Breg->index;
    level = cuddI(dd,id);
    t = zddPortFromBddStep(dd, Bt, level+1);
    if (t == NULL) return(NULL);
    cuddRef(t);
    e = zddPortFromBddStep(dd, Be, level+1);
    if (e == NULL) {
	Cudd_RecursiveDerefZdd(dd, t);
	return(NULL);
    }
    cuddRef(e);
    res = cuddZddGetNode(dd, id, t, e);
    if (res == NULL) {
	Cudd_RecursiveDerefZdd(dd, t);
	Cudd_RecursiveDerefZdd(dd, e);
	return(NULL);
    }
    cuddRef(res);
    Cudd_RecursiveDerefZdd(dd, t);
    Cudd_RecursiveDerefZdd(dd, e);

    cuddCacheInsert1(dd,Cudd_zddPortFromBdd,B,res);

    for (level--; level >= expected; level--) {
	prevZdd = res;
	id = dd->invperm[level];
	res = cuddZddGetNode(dd, id, prevZdd, prevZdd);
	if (res == NULL) {
	    Cudd_RecursiveDerefZdd(dd, prevZdd);
	    return(NULL);
	}
	cuddRef(res);
	Cudd_RecursiveDerefZdd(dd, prevZdd);
    }

    cuddDeref(res);
    return(res);

} /* end of zddPortFromBddStep */


/**
  @brief Performs the recursive step of Cudd_zddPortFromBddNegCof().

  @sideeffect None

*/
static DdNode *
zddPortFromBddNegCofStep(
  DdManager * dd /**< manager */,
  DdNode * B /**< %BDD to be converted */,
  DdNode * cube /**< cube of cofactor variables */,
  int * indices /**< lookup table of cofactor variables */,
  int expected /**< expected recursion depth */)
{
    DdNode	*res, *prevZdd, *t, *e;
    DdNode	*Breg, *Bt, *Be;
    int		id;
    int		level;

    statLine(dd);
    /* Terminal cases. */
    if (B == Cudd_Not(DD_ONE(dd)))
	return(DD_ZERO(dd));
    if (B == DD_ONE(dd)) {
        res = DD_ONE(dd);
        cuddRef(res);
        level = dd->size;
        for (level--; level >= expected; level--) {
            id = dd->invperm[level];
            if (indices[id] == 0) {
                prevZdd = res;
                res = cuddZddGetNode(dd, id, prevZdd, prevZdd);
                if (res == NULL) {
                    Cudd_RecursiveDerefZdd(dd, prevZdd);
                    return(NULL);
                }
                cuddRef(res);
                Cudd_RecursiveDerefZdd(dd, prevZdd);
            }
        }
        cuddDeref(res);
        return(res);
    }

    Breg = Cudd_Regular(B);

    /* Computed table look-up. */
    res = cuddCacheLookup2Zdd(dd,Cudd_zddPortFromBddNegCof,B,cube);
    if (res != NULL) {
	level = cuddI(dd,Breg->index);
	/* Adding DC vars. */
	if (expected < level) {
	    /* Add suppressed variables. */
	    cuddRef(res);
	    for (level--; level >= expected; level--) {
		id = dd->invperm[level];
                if (indices[id] == 0) {
                    prevZdd = res;
                    res = cuddZddGetNode(dd, id, prevZdd, prevZdd);
                    if (res == NULL) {
                        Cudd_RecursiveDerefZdd(dd, prevZdd);
                        return(NULL);
                    }
                    cuddRef(res);
                    Cudd_RecursiveDerefZdd(dd, prevZdd);
                }
	    }
	    cuddDeref(res);
	}
	return(res);
    }	/* end of cache look-up */

    if (Cudd_IsComplement(B)) {
	Bt = Cudd_Not(cuddT(Breg));
	Be = Cudd_Not(cuddE(Breg));
    } else {
	Bt = cuddT(Breg);
	Be = cuddE(Breg);
    }

    id = (int) Breg->index;
    level = cuddI(dd,id);
    if (indices[id] == 1) {
        /* The BDD should not depend on the variables in indices. */
        dd->errorCode = CUDD_INVALID_ARG;
        return(NULL);
    }
    t = zddPortFromBddNegCofStep(dd, Bt, cube, indices, level+1);
    if (t == NULL) return(NULL);
    cuddRef(t);
    e = zddPortFromBddNegCofStep(dd, Be, cube, indices, level+1);
    if (e == NULL) {
	Cudd_RecursiveDerefZdd(dd, t);
	return(NULL);
    }
    cuddRef(e);
    res = cuddZddGetNode(dd, id, t, e);
    if (res == NULL) {
	Cudd_RecursiveDerefZdd(dd, t);
	Cudd_RecursiveDerefZdd(dd, e);
	return(NULL);
    }
    cuddRef(res);
    Cudd_RecursiveDerefZdd(dd, t);
    Cudd_RecursiveDerefZdd(dd, e);

    cuddCacheInsert2(dd,Cudd_zddPortFromBddNegCof,B,cube,res);

    for (level--; level >= expected; level--) {
	id = dd->invperm[level];
        if (indices[id] == 0) {
            prevZdd = res;
            res = cuddZddGetNode(dd, id, prevZdd, prevZdd);
            if (res == NULL) {
                Cudd_RecursiveDerefZdd(dd, prevZdd);
                return(NULL);
            }
            cuddRef(res);
            Cudd_RecursiveDerefZdd(dd, prevZdd);
        }
    }

    cuddDeref(res);
    return(res);

} /* end of zddPortFromBddNegCofStep */


/**
  @brief Performs the recursive step of Cudd_zddPortToBdd().

  @sideeffect None

*/
static DdNode *
zddPortToBddStep(
  DdManager * dd /**< manager */,
  DdNode * f /**< %ZDD to be converted */,
  int  depth /**< recursion depth */)
{
    DdNode *one, *zero, *T, *E, *res, *var;
    int index;
    int level;

    statLine(dd);
    one = DD_ONE(dd);
    zero = DD_ZERO(dd);
    if (f == zero) return(Cudd_Not(one));

    if (depth == dd->sizeZ) return(one);

    index = dd->invpermZ[depth];
    level = cuddIZ(dd,f->index);
    var = dd->vars[index];

    if (level > depth) {
	E = zddPortToBddStep(dd,f,depth+1);
	if (E == NULL) {
	    return(NULL);
	}
	cuddRef(E);
	res = cuddBddIteRecur(dd,var,Cudd_Not(one),E);
	if (res == NULL) {
	    Cudd_RecursiveDeref(dd,E);
	    return(NULL);
	}
	cuddRef(res);
	Cudd_RecursiveDeref(dd,E);
	cuddDeref(res);
	return(res);
    }

    res = cuddCacheLookup1(dd,Cudd_zddPortToBdd,f);
    if (res != NULL) {
	return(res);
    }

    T = zddPortToBddStep(dd,cuddT(f),depth+1);
    if (T == NULL) {
	return(NULL);
    }
    cuddRef(T);
    E = zddPortToBddStep(dd,cuddE(f),depth+1);
    if (E == NULL) {
	Cudd_RecursiveDeref(dd,T);
	return(NULL);
    }
    cuddRef(E);

    res = cuddBddIteRecur(dd,var,T,E);
    if (res == NULL) {
	Cudd_RecursiveDeref(dd,T);
	Cudd_RecursiveDeref(dd,E);
	return(NULL);
    }
    cuddRef(res);
    Cudd_RecursiveDeref(dd,T);
    Cudd_RecursiveDeref(dd,E);
    cuddDeref(res);

    cuddCacheInsert1(dd,Cudd_zddPortToBdd,f,res);

    return(res);

} /* end of zddPortToBddStep */


/**
  @brief Performs the recursive step of Cudd_zddPortToBddNegCof().

  @sideeffect None

*/
static DdNode *
zddPortToBddNegCofStep(
  DdManager * dd /**< manager */,
  DdNode * f /**< %ZDD to be converted */,
  DdNode *cube /**< cube of cofactor variables */,
  int *indices /**< lookup table of cofactor variables */,
  int  depth /**< recursion depth */)
{
    DdNode *one, *zero, *T, *E, *res, *var;
    int index;
    int level;

    statLine(dd);
    one = DD_ONE(dd);
    zero = DD_ZERO(dd);
    if (f == zero) return(Cudd_Not(one));

    if (depth == dd->sizeZ) return(one);

    index = dd->invpermZ[depth];
    level = cuddIZ(dd,f->index);
    var = dd->vars[index];

    if (level > depth) {
        E = zddPortToBddNegCofStep(dd,f,cube,indices,depth+1);
	if (E == NULL) {
	    return(NULL);
	}
        if (indices[index] == 1) {
            res = E;
        } else {
            cuddRef(E);
            res = cuddBddIteRecur(dd,var,Cudd_Not(one),E);
            if (res == NULL) {
                Cudd_RecursiveDeref(dd,E);
                return(NULL);
            }
            cuddRef(res);
            Cudd_RecursiveDeref(dd,E);
            cuddDeref(res);
        }
	return(res);
    }

    res = cuddCacheLookup2(dd,Cudd_zddPortToBddNegCof,f,cube);
    if (res != NULL) {
	return(res);
    }

    T = zddPortToBddNegCofStep(dd,cuddT(f),cube,indices,depth+1);
    if (T == NULL) {
	return(NULL);
    }
    cuddRef(T);
    E = zddPortToBddNegCofStep(dd,cuddE(f),cube,indices,depth+1);
    if (E == NULL) {
	Cudd_RecursiveDeref(dd,T);
	return(NULL);
    }
    cuddRef(E);

    res = cuddBddIteRecur(dd,var,T,E);
    if (res == NULL) {
	Cudd_RecursiveDeref(dd,T);
	Cudd_RecursiveDeref(dd,E);
	return(NULL);
    }
    cuddRef(res);
    Cudd_RecursiveDeref(dd,T);
    Cudd_RecursiveDeref(dd,E);
    cuddDeref(res);

    cuddCacheInsert2(dd,Cudd_zddPortToBddNegCof,f,cube,res);

    return(res);

} /* end of zddPortToBddNegCofStep */
