%% Clear and initialize symbolic pap and anchors
clear; clc;
syms p y par tmx tmy tmz tax tay taz tox toy toz real
pap = [p y par]';
tm = [tmx tmy tmz]';
ta = [tax tay taz]';
to = [tox toy toz]';

%% VectorFromMain
vecFromMain = [cos(p)*cos(y) cos(p)*sin(y) sin(p)]';

VECFROMMAIN_pap = simplify(jacobian(vecFromMain,pap));

%% VectorFromAsso
tmta = ta - tm;

% NOTE: There are two ways to compute phi, each one of them present a
% discontinuity on the jacobians on different points. Check the commented
% section below for examples where the jacobians fails

% When using acos, the jacobians are discontinuous if phi -> 0 or phi -> pi.
% Note that phi -> 0 can happen easily in a incremental forward looking
% camera scenario.
phi = acos( dot(vecFromMain,tmta/norm(tmta)));

% % When using atan2 the jacobians will be discontinuous if phi -> pi/2.
% % Note that this most likely to happen in the side-looking scenario.
% phi = atan2( norm(cross(vecFromMain,tmta)), dot(vecFromMain,tmta));

vecFromAsso = sin(par + phi)*norm(tmta)*vecFromMain - sin(par)*tmta;

VECFROMASSO_tm  = simplify(jacobian(vecFromAsso,tm));
VECFROMASSO_ta  = simplify(jacobian(vecFromAsso,ta));
VECFROMASSO_pap = simplify(jacobian(vecFromAsso,pap));

%% VectorFromOthe
tmto = to - tm;

vecFromOthe = sin(par + phi)*norm(tmta)*vecFromMain - sin(par)*tmto;

VECFROMOTHE_tm  = simplify(jacobian(vecFromOthe,tm));
VECFROMOTHE_ta  = simplify(jacobian(vecFromOthe,ta));
VECFROMOTHE_to  = simplify(jacobian(vecFromOthe,to));
VECFROMOTHE_pap = simplify(jacobian(vecFromOthe,pap));

%% Values used in the test (3D point is at origin)
tmx = 0; tmy = -3; tmz = 0;
tax = 0; tay = -5; taz = 5;
tox = 7; toy =  0; toz = 0;
p = 0; y = deg2rad(90.0); par = deg2rad(45.0);

%% Evaluate jacobians from VecFromMain 
eVECFROMMAIN_pap = eval(VECFROMMAIN_pap)

%% Evaluate jacobians from VecFromAsso
eVECFROMASSO_pap = eval(VECFROMASSO_pap)
eVECFROMASSO_tm  = eval(VECFROMASSO_tm)
eVECFROMASSO_ta  = eval(VECFROMASSO_ta)

%% Evaluate jacobians from VecFromOthe
eVECFROMOTHE_pap = eval(VECFROMOTHE_pap)
eVECFROMOTHE_tm = eval(VECFROMOTHE_tm)
eVECFROMOTHE_ta = eval(VECFROMOTHE_ta)
eVECFROMOTHE_to = eval(VECFROMOTHE_to)

%% Other examples to test/show the discontinuity in the jacobians computation

% %% New values to compute jacobians when phi -> 0
% % Should not work if phi is computed using 'acos' formula.
% tmx = 0; tmy = -3; tmz = 0;
% tax = 0; tay = -5; taz = 0;
% tox = 7; toy =  0; toz = 0;
% p = 0; y = deg2rad(90.0); par = deg2rad(0.0);
% 
% %% Evaluate jacobians from VecFromMain 
% eVECFROMMAIN_pap = eval(VECFROMMAIN_pap)
% 
% %% Evaluate jacobians from VecFromAsso
% eVECFROMASSO_pap = eval(VECFROMASSO_pap)
% eVECFROMASSO_tm  = eval(VECFROMASSO_tm)
% eVECFROMASSO_ta  = eval(VECFROMASSO_ta)
% 
% %% Evaluate jacobians from VecFromOthe
% eVECFROMOTHE_pap = eval(VECFROMOTHE_pap)
% eVECFROMOTHE_tm = eval(VECFROMOTHE_tm)
% eVECFROMOTHE_ta = eval(VECFROMOTHE_ta)
% eVECFROMOTHE_to = eval(VECFROMOTHE_to)
% 
% %% New values to compute jacobians when phi -> pi/2
% % Should not work if phi is computed using 'atan2' formula.
% tmx = 0; tmy = -3; tmz = 0;
% tax = 0; tay = -3; taz = 3;
% tox = 7; toy =  0; toz = 0;
% p = 0; y = deg2rad(90.0); par = deg2rad(45.0);
% 
% %% Evaluate jacobians from VecFromMain 
% eVECFROMMAIN_pap = eval(VECFROMMAIN_pap)
% 
% %% Evaluate jacobians from VecFromAsso
% eVECFROMASSO_pap = eval(VECFROMASSO_pap)
% eVECFROMASSO_tm  = eval(VECFROMASSO_tm)
% eVECFROMASSO_ta  = eval(VECFROMASSO_ta)
% 
% %% Evaluate jacobians from VecFromOthe
% eVECFROMOTHE_pap = eval(VECFROMOTHE_pap)
% eVECFROMOTHE_tm = eval(VECFROMOTHE_tm)
% eVECFROMOTHE_ta = eval(VECFROMOTHE_ta)
% eVECFROMOTHE_to = eval(VECFROMOTHE_to)
% 
