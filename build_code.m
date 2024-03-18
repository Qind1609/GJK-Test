%% create log
delete build_log.txt;
diary('build_log.txt');
diary on;

%% setup compiler
% uncomment mex command below and change the path to setup your compiler.
% Some tricks has been performed to compile .cpp file with msvcpp2022 compiler.
%(Matlab 2020a is not compatible with this compiler of VS2022) 

mex -setup:'C:\Program Files\Polyspace\R2020a\bin\win64\mexopts\msvcpp2022.xml' C++ -v


%% setup header include and libraries path.

% change these below paths to your own paths on your computer
% Note that CGAL 5.6.1 needs boost version 1.66 (or later) 
% and C++14 (or later)
cgal_include = "C:\CGAL-5.6.1\include";
boost_include = "C:\Boost\boost_1_84_0";
boost_lib = "C:\Boost\boost_1_84_0\stage\lib";


mex('-v','-I' + cgal_include, '-I' + boost_include,'./PolylineSimplification/polylineSimplify.cpp','-L'+boost_lib);
mex('-v','-I' + cgal_include, '-I' + boost_include,'./PolygonDecomposition/convexPartitioning.cpp','-L'+boost_lib);
diary off;