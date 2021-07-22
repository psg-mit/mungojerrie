AM_CFLAGS =
AM_CXXFLAGS = -std=c++11 $(AM_CFLAGS)

noinst_LIBRARIES = libutil.a libst.a libmtr.a libepd.a libcudd.a libobj.a

libutil_a_SOURCES = cpu_stats.c cpu_time.c datalimit.c \
  prtime.c safe_mem.c ucbqsort.c util.h cstringstream.c

libst_a_SOURCES = st.c

libmtr_a_SOURCES = mtrBasic.c mtrGroup.c

libepd_a_SOURCES = epd.c

libcudd_a_SOURCES = \
  cuddAPI.c cuddAddAbs.c cuddAddApply.c cuddAddFind.c cuddAddIte.c \
  cuddAddInv.c cuddAddNeg.c cuddAddWalsh.c cuddAndAbs.c \
  cuddAnneal.c cuddApa.c cuddApprox.c cuddBddAbs.c cuddBddCorr.c \
  cuddBddIte.c cuddBridge.c cuddCache.c cuddCheck.c cuddClip.c \
  cuddCof.c cuddCompose.c cuddDecomp.c cuddEssent.c \
  cuddExact.c cuddExport.c cuddGenCof.c cuddGenetic.c \
  cuddGroup.c cuddHarwell.c cuddInit.c cuddInteract.c \
  cuddLCache.c cuddLevelQ.c \
  cuddLinear.c cuddLiteral.c cuddMatMult.c cuddPriority.c \
  cuddRead.c cuddRef.c cuddReorder.c cuddSat.c cuddSign.c \
  cuddSolve.c cuddSplit.c cuddSubsetHB.c cuddSubsetSP.c cuddSymmetry.c \
  cuddTable.c cuddUtil.c cuddWindow.c cuddZddCount.c cuddZddFuncs.c \
  cuddZddGroup.c cuddZddIsop.c cuddZddLin.c cuddZddMisc.c \
  cuddZddPort.c cuddZddReord.c cuddZddSetop.c cuddZddSymm.c \
  cuddZddUtil.c

libobj_a_SOURCES = cuddObj.cc 

include_HEADERS = \
  st.h \
  cudd.h \
  cuddInt.h \
  cuddObj.hh \
  epdInt.h \
  epd.h \
  mtr.h \
  mtrInt.h \
  cstringstream.h

check_PROGRAMS = testmtr testcudd testextra testobj
check_SCRIPTS = test_mtr.test test_cudd.test

testmtr_SOURCES = testmtr.c
testmtr_LDADD = libmtr.a libutil.a

testcudd_SOURCES = testcudd.c
testcudd_LDADD = libcudd.a libst.a libepd.a $(testmtr_LDADD)

testextra_SOURCE = testextra.c
testextra_LDADD = $(testcudd_LDADD)

testobj_SOURCES = testobj.cc
testobj_LDADD = libobj.a $(testcudd_LDADD)

dist_check_DATA = test.groups r7x8.1.mat
EXTRA_DIST = test_mtr.test.in test_cudd.test.in
CLEANFILES = $(check_SCRIPTS) r7x8.1.tst extra.tst differences

do_subst = sed \
  -e 's,[@]EXEEXT[@],$(EXEEXT),g' \
  -e 's,[@]srcdir[@],$(srcdir),g'

test_mtr.test: test_mtr.test.in Makefile
	$(do_subst) $< > $@
	chmod +x $@

test_cudd.test: test_cudd.test.in Makefile
	$(do_subst) $< > $@
	chmod +x $@

TESTS = $(check_SCRIPTS) testobj

@VALGRIND_CHECK_RULES@
VALGRIND_SUPPRESSIONS_FILES =
