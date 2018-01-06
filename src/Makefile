LIB_PATHS = -L$(CPLEX_HOME)/cplex/lib/x86-64_sles10_4.1/static_pic -L$(CPLEX_HOME)/concert/lib/x86-64_sles10_4.1/static_pic

INCLUDE_PATHS = -I$(CPLEX_HOME)/cplex/include -I$(CPLEX_HOME)/concert/include
LIBS = -lilocplex -lconcert -lcplex -lm -lpthread
DFLAGS = -DIL_STD -DCPLEX_PZ
FILES = middleman.cc

all:
	g++ -O3 -std=c++0x $(DFLAGS) $(LIB_PATHS) $(INCLUDE_PATHS) $(FILES) $(LIBS) -o middleman

dbg:
	g++ -DDBG -g -std=c++0x $(DFLAGS) $(LIB_PATHS) $(INCLUDE_PATHS) $(FILES) $(LIBS) -o middleman

debug:
	g++ -g -std=c++0x $(DFLAGS) $(LIB_PATHS) $(INCLUDE_PATHS) $(FILES) $(LIBS) -o middleman
