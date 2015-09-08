%% Clear and initialize symbolic pap and anchors
clear; clc;
syms p y par real
syms mtx mty mtz mex mey mez real
syms atx aty atz aex aey aez real
syms otx oty otz oex oey oez real
% syms stx sty stz sex sey sez sqw sqx sqy sqz real
syms u0 v0 au av real

pap = [p y par]';
mt = [mtx mty mtz]'; mq = e2q([mex mey mez]');
at = [atx aty atz]'; aq = e2q([aex aey aez]');
ot = [otx oty otz]'; oq = e2q([oex oey oez]');
% st = [stx sty stz]'; sq = e2q([sex sey sez]');

mainFrame.x = [mt; mq];
assoFrame.x = [at; aq];
otheFrame.x = [ot; oq];
% sensorFrame.x = [st; sq];

mainFrame = updateFrame(mainFrame);
assoFrame = updateFrame(assoFrame);
otheFrame = updateFrame(otheFrame);
% sensorFrame = updateFrame(sensorFrame);

gtsamMainFrame = [mex mey mez mtx mty mtz]';
gtsamAssoFrame = [aex aey aez atx aty atz]';
gtsamOtheFrame = [oex oey oez otx oty otz]';

K = [u0;v0;au;av];

%% Projection to main anchor
% compose camera frame
%mainCamFrame = composeFrames(mainFrame,sensorFrame);
mainCamFrame = mainFrame;

% Get vector from main
vecFromMain = [cos(pap(1))*cos(pap(2)) cos(pap(1))*sin(pap(2)) sin(pap(1))]';

% Unrotate
pc = mainCamFrame.Rt*vecFromMain;

% project to camera
pn = [pc(1)/pc(3) pc(2)/pc(3)]';

% uncalibrate
pim = [(K(3) * pn(1) + K(1)) (K(4) * pn(2) + K(2))]';

PIMMAIN_pap = jacobian(pim,pap);
PIMMAIN_gtmain = jacobian(pim,gtsamMainFrame);

%% Projection to asso anchor
% compose camera frame
mainCamFrame = mainFrame;
assoCamFrame = assoFrame;

% Get vector from main
vecFromMain = [cos(p)*cos(y) cos(p)*sin(y) sin(p)]';

% Vector from main anchor to asso anchor
tmta = assoCamFrame.t - mainCamFrame.t;

% Angle between vecFromMain and tmta
%phi = atan2( norm(cross(vecFromMain,tmta)), dot(vecFromMain,tmta));
phi = acos( dot(vecFromMain,tmta/norm(tmta)));

% Get vector from asso
vecFromAsso = sin(pap(3) + phi)*norm(tmta)*vecFromMain - sin(pap(3))*tmta;

% Unrotate
pc = assoCamFrame.Rt*vecFromAsso;

% project to camera
pn = [pc(1)/pc(3) pc(2)/pc(3)]';

% uncalibrate
pim = [(K(3) * pn(1) + K(1)) (K(4) * pn(2) + K(2))]';

PIMASSO_pap = jacobian(pim,pap);
PIMASSO_gtmain = jacobian(pim,gtsamMainFrame);
PIMASSO_gtasso = jacobian(pim,gtsamAssoFrame);

%% Projection to other
% compose camera frame
mainCamFrame = mainFrame;
assoCamFrame = assoFrame;
otheCamFrame = otheFrame;

% Get vector from main
vecFromMain = [cos(p)*cos(y) cos(p)*sin(y) sin(p)]';

% Vector from main anchor to asso anchor
tmta = assoCamFrame.t - mainCamFrame.t;

% Vector from main anchor to other
tmto = otheCamFrame.t - mainCamFrame.t;

% Angle between vecFromMain and tmta
%phi = atan2( norm(cross(vecFromMain,tmta)), dot(vecFromMain,tmta));
phi = acos( dot(vecFromMain,tmta/norm(tmta)));

% Get vector from other
vecFromOthe = sin(pap(3) + phi)*norm(tmta)*vecFromMain - sin(pap(3))*tmto;

% Unrotate
pc = otheCamFrame.Rt*vecFromOthe;

% project to camera
pn = [pc(1)/pc(3) pc(2)/pc(3)]';

% uncalibrate
pim = [(K(3) * pn(1) + K(1)) (K(4) * pn(2) + K(2))]';

PIMOTHE_pap = jacobian(pim,pap);
PIMOTHE_gtmain = jacobian(pim,gtsamMainFrame);
PIMOTHE_gtasso = jacobian(pim,gtsamAssoFrame);
PIMOTHE_gtothe = jacobian(pim,gtsamOtheFrame);

%% Values used in the test main projection (3D point is at origin)
p = deg2rad(90); y = deg2rad(0); par = deg2rad(0);
mtx =  0; mty = 0; mtz =  -6; mex = 0; mey = 0; mez = 0; 
% atx = -6; aty =  0; atz =  0; aex = 0; aey = 0; aez = 0; 
% otx =  0; oty =  0; otz = -6; oex = 0; oey = 0; oez = 0; 
u0 = 320; v0 = 240; au = 554.256; av = 554.256;

%% Evaluate jacobians for main projection

ePIMMAIN_pap = eval(PIMMAIN_pap)
ePIMMAIN_gtmain = eval(PIMMAIN_gtmain)

%% Values used in the test asso projection (3D point is at origin)
p = deg2rad(0); y = deg2rad(90); par = deg2rad(90);
mtx =  0; mty = -6; mtz =  0; mex = 0; mey = 0; mez = 0; 
atx =  0; aty =  0; atz = -6; aex = 0; aey = 0; aez = 0; 
% otx =  0; oty =  0; otz = -6; oex = 0; oey = 0; oez = 0; 

%% Evaluate jacobians for asso projection

ePIMASSO_pap = eval(PIMASSO_pap)
ePIMASSO_gtmain = eval(PIMASSO_gtmain)
ePIMASSO_gtasso = eval(PIMASSO_gtasso)

%% Values used in the test other projection (3D point is at origin)
p = deg2rad(0); y = deg2rad(90); par = deg2rad(90);
mtx =  0; mty = -6; mtz =  0; mex = 0; mey = 0; mez = 0; 
atx = -6; aty =  0; atz =  0; aex = 0; aey = 0; aez = 0; 
otx =  0; oty =  0; otz = -6; oex = 0; oey = 0; oez = 0; 

%% Evaluate jacobians for other projection

ePIMOTHE_pap = eval(PIMOTHE_pap)
ePIMOTHE_gtmain = eval(PIMOTHE_gtmain)
ePIMOTHE_gtasso = eval(PIMOTHE_gtasso)
ePIMOTHE_gtothe = eval(PIMOTHE_gtothe)
