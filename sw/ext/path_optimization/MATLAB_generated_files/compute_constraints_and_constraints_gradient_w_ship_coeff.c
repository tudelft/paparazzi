/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_constraints_and_constraints_gradient_w_ship_coeff.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "compute_constraints_and_constraints_gradient_w_ship_coeff.h"
#include "path_optimizer_fcn_rtwutil.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Type Definitions */
#ifndef typedef_cell_14
#define typedef_cell_14

typedef struct {
  double f1;
  double f2;
  double f3;
  double f4;
  double f5;
  double f6;
  double f7;
  double f8;
  double f9;
  double f10;
  double f11;
  double f12;
  double f13;
  double f14;
  double f15;
  double f16;
  double f17;
  double f18;
  double f19;
  double f20;
  double f21;
  double f22;
  double f23;
  double f24;
  double f25;
  double f26;
  double f27;
  double f28;
  double f29;
  double f30;
  double f31;
  double f32;
  double f33;
  double f34;
  double f35;
  double f36;
  double f37;
  double f38;
  double f39;
  double f40;
  double f41;
  double f42;
  double f43;
  double f44;
  double f45;
  double f46;
  double f47;
  double f48;
  double f49;
  double f50;
  double f51;
  double f52;
  double f53;
  double f54;
  double f55;
  double f56;
  double f57;
  double f58;
  double f59;
  double f60;
  double f61;
  double f62;
  double f63;
  double f64;
  double f65;
  double f66;
  double f67;
  double f68;
  double f69;
  double f70;
  double f71;
  double f72;
  double f73;
  double f74;
  double f75;
  double f76;
  double f77;
  double f78;
  double f79;
  double f80;
  double f81;
  double f82;
  double f83;
  double f84;
  double f85;
  double f86;
  double f87;
  double f88;
  double f89;
  double f90;
  double f91;
  double f92;
  double f93;
  double f94;
  double f95;
  double f96;
  double f97;
  double f98;
  double f99;
  double f100;
  double f101;
  double f102;
  double f103;
  double f104;
  double f105;
  double f106;
  double f107;
  double f108;
  double f109;
  double f110;
  double f111;
  double f112;
  double f113;
  double f114;
  double f115;
  double f116;
  double f117;
  double f118;
  double f119;
  double f120;
  double f121;
  double f122;
  double f123;
  double f124;
  double f125;
  double f126;
  double f127;
  double f128;
  double f129;
  double f130;
  double f131;
  double f132;
  double f133;
  double f134;
  double f135;
  double f136;
  creal_T f137;
  double f138;
  double f139;
  double f140;
  double f141;
  double f142;
  double f143;
  double f144;
  double f145;
  double f146;
  double f147;
  double f148;
  double f149;
  double f150;
  double f151;
  double f152;
  double f153;
  double f154;
  double f155;
  double f156;
  double f157;
  double f158;
  creal_T f159;
  creal_T f160;
  creal_T f161;
  creal_T f162;
  creal_T f163;
  creal_T f164;
  creal_T f165;
  creal_T f166;
  creal_T f167;
  double f168;
  double f169;
  double f170;
  double f171;
  double f172;
  double f173;
  double f174;
  double f175;
  double f176;
  double f177;
  double f178;
  double f179;
  double f180;
  double f181;
  double f182;
  double f183;
  double f184;
  double f185;
  double f186;
  double f187;
  double f188;
  double f189;
  double f190;
  double f191;
  double f192;
  double f193;
  double f194;
  double f195;
  double f196;
  double f197;
  double f198;
  double f199;
  double f200;
  double f201;
  double f202;
  double f203;
  double f204;
  double f205;
  double f206;
  double f207;
  double f208;
  double f209;
  double f210;
  double f211;
  double f212;
  double f213;
  double f214;
  double f215;
  double f216;
  double f217;
  double f218;
  double f219;
  double f220;
  double f221;
  double f222;
  double f223;
  double f224;
  double f225;
  double f226;
  double f227;
  double f228;
  double f229;
  double f230;
  double f231;
  double f232;
  double f233;
  double f234;
  double f235;
  double f236;
  double f237;
  double f238;
  double f239;
  double f240;
  double f241;
  double f242;
  double f243;
  double f244;
  double f245;
  double f246;
  double f247;
  double f248;
  double f249;
  double f250;
  double f251;
  double f252;
  double f253;
  double f254;
  double f255;
  double f256;
  double f257;
  double f258;
  double f259;
  double f260;
  double f261;
  double f262;
  double f263;
  double f264;
  double f265;
  double f266;
  double f267;
  double f268;
  double f269;
  double f270;
  double f271;
  double f272;
  double f273;
  double f274;
  double f275;
  double f276;
  double f277;
  creal_T f278;
  creal_T f279;
  creal_T f280;
  creal_T f281;
  creal_T f282;
  creal_T f283;
  creal_T f284;
  creal_T f285;
  creal_T f286;
  double f287;
  creal_T f288;
  double f289;
  double f290;
  double f291;
  double f292;
  double f293;
  double f294;
  double f295;
  double f296;
  double f297;
  double f298;
  double f299;
  double f300;
  double f301;
  double f302;
  double f303;
  double f304;
  double f305;
  double f306;
  double f307;
  double f308;
  double f309;
  double f310;
  double f311;
  double f312;
  double f313;
  double f314;
  double f315;
  double f316;
  double f317;
  double f318;
  double f319;
  double f320;
  double f321;
  double f322;
  double f323;
  double f324;
  double f325;
  double f326;
  double f327;
  double f328;
  double f329;
  double f330;
  double f331;
  double f332;
  double f333;
  double f334;
  double f335;
  double f336;
  double f337;
  double f338;
  double f339;
  double f340;
  double f341;
  double f342;
  double f343;
  double f344;
  double f345;
  double f346;
  creal_T f347;
  creal_T f348;
  creal_T f349;
  creal_T f350;
  creal_T f351;
  creal_T f352;
  double f353;
  creal_T f354;
  creal_T f355;
  creal_T f356;
  creal_T f357;
  creal_T f358;
  creal_T f359;
  creal_T f360;
  creal_T f361;
  creal_T f362;
  creal_T f363;
  double f364;
  creal_T f365;
  creal_T f366;
  creal_T f367;
  creal_T f368;
  creal_T f369;
  creal_T f370;
  creal_T f371;
  creal_T f372;
  creal_T f373;
  creal_T f374;
  double f375;
  creal_T f376;
  creal_T f377;
  creal_T f378;
  creal_T f379;
  creal_T f380;
  creal_T f381;
  creal_T f382;
  creal_T f383;
  creal_T f384;
  creal_T f385;
  double f386;
  creal_T f387;
  creal_T f388;
  creal_T f389;
  creal_T f390;
  creal_T f391;
  creal_T f392;
  creal_T f393;
  creal_T f394;
  creal_T f395;
  creal_T f396;
  double f397;
  creal_T f398;
  creal_T f399;
  creal_T f400;
  creal_T f401;
  creal_T f402;
  creal_T f403;
  creal_T f404;
  creal_T f405;
  double f406;
  double f407;
  double f408;
  double f409;
  double f410;
  double f411;
  double f412;
  double f413;
  double f414;
  double f415;
  double f416;
  double f417;
  double f418;
  double f419;
  double f420;
  double f421;
  double f422;
  double f423;
  double f424;
  double f425;
  double f426;
  double f427;
  double f428;
  double f429;
  double f430;
  double f431;
  double f432;
  double f433;
  double f434;
  double f435;
  double f436;
  double f437;
  double f438;
  double f439;
  double f440;
  double f441;
  double f442;
  double f443;
  double f444;
  double f445;
  double f446;
  double f447;
  double f448;
  double f449;
  double f450;
  double f451;
  double f452;
  double f453;
  double f454;
  double f455;
  double f456;
  double f457;
  double f458;
  double f459;
  double f460;
  double f461;
  double f462;
  double f463;
  double f464;
  double f465;
  double f466;
  double f467;
  double f468;
  double f469;
  double f470;
  double f471;
  double f472;
  double f473;
  double f474;
  double f475;
  double f476;
  double f477;
  double f478;
  double f479;
  double f480;
  double f481;
  double f482;
  double f483;
  double f484;
  double f485;
  double f486;
  double f487;
  double f488;
  double f489;
  double f490;
  double f491;
} cell_14;

#endif                                 /* typedef_cell_14 */

#ifndef typedef_cell_15
#define typedef_cell_15

typedef struct {
  double f1;
  double f2;
  double f3;
  double f4;
  double f5;
  double f6;
  double f7;
  double f8;
  double f9;
  double f10;
  double f11;
  double f12;
  double f13;
  double f14;
  double f15;
  double f16;
  double f17;
  double f18;
  double f19;
  double f20;
  double f21;
  double f22;
  double f23;
  double f24;
  double f25;
  double f26;
  double f27;
  double f28;
  double f29;
  double f30;
  double f31;
  double f32;
  double f33;
  double f34;
  double f35;
  double f36;
  double f37;
  double f38;
  double f39;
  double f40;
  double f41;
  double f42;
  double f43;
  double f44;
  double f45;
  double f46;
  double f47;
  double f48;
  double f49;
  double f50;
  double f51;
  double f52;
  double f53;
  double f54;
  double f55;
  double f56;
  double f57;
  double f58;
  double f59;
  double f60;
  double f61;
  double f62;
  double f63;
  double f64;
  double f65;
  double f66;
  double f67;
  double f68;
  double f69;
  double f70;
  double f71;
  double f72;
  double f73;
  double f74;
  double f75;
  double f76;
  double f77;
  double f78;
  double f79;
  double f80;
  double f81;
  double f82;
  double f83;
  double f84;
  double f85;
  double f86;
  double f87;
  double f88;
  double f89;
  double f90;
  double f91;
  double f92;
  double f93;
  double f94;
  double f95;
  double f96;
  double f97;
  double f98;
  double f99;
  double f100;
  double f101;
  double f102;
  double f103;
  double f104;
  double f105;
  double f106;
  double f107;
  double f108;
  double f109;
  double f110;
  double f111;
  double f112;
  double f113;
  double f114;
  double f115;
  double f116;
  double f117;
  double f118;
  double f119;
  double f120;
  double f121;
  double f122;
  double f123;
  double f124;
  double f125;
  double f126;
  double f127;
  double f128;
  double f129;
  double f130;
  double f131;
  double f132;
  double f133;
  double f134;
  double f135;
  double f136;
  double f137;
  double f138;
  double f139;
  double f140;
  double f141;
  double f142;
  double f143;
  double f144;
  double f145;
  double f146;
  double f147;
  double f148;
  double f149;
  double f150;
  double f151;
  double f152;
  double f153;
  double f154;
  double f155;
  double f156;
  double f157;
  double f158;
  double f159;
  double f160;
  double f161;
  double f162;
  double f163;
  double f164;
  double f165;
  double f166;
  double f167;
  double f168;
  creal_T f169;
  creal_T f170;
  creal_T f171;
  creal_T f172;
  creal_T f173;
  creal_T f174;
  creal_T f175;
  creal_T f176;
  creal_T f177;
  creal_T f178;
  double f179;
  creal_T f180;
  creal_T f181;
  creal_T f182;
  creal_T f183;
  creal_T f184;
  creal_T f185;
  creal_T f186;
  creal_T f187;
  creal_T f188;
  creal_T f189;
  double f190;
  creal_T f191;
  creal_T f192;
  creal_T f193;
  creal_T f194;
  creal_T f195;
  creal_T f196;
  creal_T f197;
  creal_T f198;
  creal_T f199;
  creal_T f200;
  double f201;
  double f202;
  creal_T f203;
  creal_T f204;
  creal_T f205;
  creal_T f206;
  creal_T f207;
  creal_T f208;
  creal_T f209;
  creal_T f210;
  creal_T f211;
  creal_T f212;
  double f213;
  creal_T f214;
  creal_T f215;
  creal_T f216;
  creal_T f217;
  creal_T f218;
  double f219;
  double f220;
  double f221;
  double f222;
  double f223;
  double f224;
  double f225;
  double f226;
  double f227;
  double f228;
  double f229;
  double f230;
  double f231;
  double f232;
  double f233;
  double f234;
  double f235;
  double f236;
  double f237;
  double f238;
  double f239;
  double f240;
  double f241;
  double f242;
  double f243;
  double f244;
  double f245;
  double f246;
  double f247;
  double f248;
  double f249;
  double f250;
  double f251;
  double f252;
  double f253;
  double f254;
  double f255;
  double f256;
  double f257;
  double f258;
  double f259;
  double f260;
  double f261;
  double f262;
  double f263;
  double f264;
  double f265;
  double f266;
  double f267;
  double f268;
  double f269;
  double f270;
  double f271;
  double f272;
  double f273;
  double f274;
  double f275;
  double f276;
  double f277;
  double f278;
  double f279;
  double f280;
  double f281;
  double f282;
  double f283;
  double f284;
  double f285;
  double f286;
  double f287;
  double f288;
  double f289;
  double f290;
  double f291;
  double f292;
  double f293;
  double f294;
  double f295;
  double f296;
  double f297;
  double f298;
  double f299;
  double f300;
  double f301;
  double f302;
  double f303;
  double f304;
  double f305;
  double f306;
  double f307;
  double f308;
  double f309;
  double f310;
  double f311;
  double f312;
  double f313;
  double f314;
  double f315;
  double f316;
  double f317;
  double f318;
  double f319;
  double f320;
  double f321;
  double f322;
  double f323;
  double f324;
  double f325;
  double f326;
  double f327;
  double f328;
  double f329;
  double f330;
  double f331;
  double f332;
  double f333;
  double f334;
  double f335;
  double f336;
  double f337;
  double f338;
  double f339;
  double f340;
  double f341;
  double f342;
  double f343;
  double f344;
  double f345;
  double f346;
  double f347;
  double f348;
  double f349;
  double f350;
  double f351;
  double f352;
  double f353;
  double f354;
  double f355;
  double f356;
  double f357;
  double f358;
  double f359;
  double f360;
  double f361;
  double f362;
  double f363;
  double f364;
  double f365;
  double f366;
  double f367;
  double f368;
  double f369;
  double f370;
  double f371;
  double f372;
  double f373;
  double f374;
  double f375;
  double f376;
  double f377;
  double f378;
  double f379;
  double f380;
  double f381;
  double f382;
  double f383;
  double f384;
  double f385;
  double f386;
  double f387;
  double f388;
  double f389;
  double f390;
  double f391;
  double f392;
  double f393;
  double f394;
  double f395;
  double f396;
  double f397;
  double f398;
  double f399;
  double f400;
  double f401;
  double f402;
  double f403;
  double f404;
  double f405;
  double f406;
  double f407;
  double f408;
  double f409;
  double f410;
  double f411;
  double f412;
  double f413;
  double f414;
  double f415;
  double f416;
  double f417;
  double f418;
  double f419;
  double f420;
  double f421;
  double f422;
  double f423;
  double f424;
  double f425;
  double f426;
  double f427;
  double f428;
  double f429;
  double f430;
  double f431;
  double f432;
  double f433;
  double f434;
  double f435;
  double f436;
  double f437;
  double f438;
  double f439;
  double f440;
  double f441;
  double f442;
  double f443;
  double f444;
  double f445;
  double f446;
  double f447;
  double f448;
  double f449;
  double f450;
  double f451;
  double f452;
  double f453;
  double f454;
  double f455;
  double f456;
  double f457;
  double f458;
  double f459;
  double f460;
  double f461;
  double f462;
  double f463;
  double f464;
  double f465;
  double f466;
  double f467;
  double f468;
  double f469;
  double f470;
  double f471;
  double f472;
  double f473;
  double f474;
  double f475;
  double f476;
  double f477;
  double f478;
  double f479;
  double f480;
  double f481;
  double f482;
  double f483;
  double f484;
  double f485;
  double f486;
  double f487;
  double f488;
  double f489;
  double f490;
  double f491;
  double f492;
  double f493;
  double f494;
  double f495;
  double f496;
  double f497;
  double f498;
  creal_T f499;
  creal_T f500;
  creal_T f501;
  creal_T f502;
  creal_T f503;
  creal_T f504;
  creal_T f505;
  double f506;
  creal_T f507;
  creal_T f508;
  double f509;
  double f510;
  double f511;
  double f512;
  creal_T f513;
  creal_T f514;
  creal_T f515;
  double f516;
  creal_T f517;
  creal_T f518;
  creal_T f519;
  creal_T f520;
  creal_T f521;
  creal_T f522;
  creal_T f523;
  creal_T f524;
  creal_T f525;
  creal_T f526;
  creal_T f527;
  creal_T f528;
  creal_T f529;
  creal_T f530;
  creal_T f531;
  creal_T f532;
  creal_T f533;
  creal_T f534;
  creal_T f535;
  creal_T f536;
  creal_T f537;
  creal_T f538;
  creal_T f539;
  creal_T f540;
  creal_T f541;
  creal_T f542;
  creal_T f543;
  creal_T f544;
  creal_T f545;
  creal_T f546;
  creal_T f547;
  creal_T f548;
  creal_T f549;
  creal_T f550;
  creal_T f551;
  creal_T f552;
  creal_T f553;
  creal_T f554;
  creal_T f555;
  creal_T f556;
  creal_T f557;
  creal_T f558;
  creal_T f559;
  creal_T f560;
  creal_T f561;
  creal_T f562;
  creal_T f563;
  creal_T f564;
  creal_T f565;
  creal_T f566;
  creal_T f567;
  creal_T f568;
  creal_T f569;
  creal_T f570;
  creal_T f571;
  creal_T f572;
  creal_T f573;
  creal_T f574;
  creal_T f575;
  creal_T f576;
  double f577;
  double f578;
  double f579;
  double f580;
  double f581;
  double f582;
  double f583;
  double f584;
  double f585;
  double f586;
  double f587;
  double f588;
  double f589;
  double f590;
  double f591;
  double f592;
  double f593;
  double f594;
  double f595;
  double f596;
  double f597;
  double f598;
  double f599;
  double f600;
  double f601;
  double f602;
  double f603;
  double f604;
  double f605;
  double f606;
  double f607;
  double f608;
  double f609;
  double f610;
  double f611;
  double f612;
  double f613;
  double f614;
  double f615;
  double f616;
  double f617;
  double f618;
  double f619;
  double f620;
  double f621;
  double f622;
  double f623;
  double f624;
  double f625;
  double f626;
  double f627;
  double f628;
  double f629;
  double f630;
  double f631;
  double f632;
  double f633;
  double f634;
  double f635;
  double f636;
  double f637;
  double f638;
  double f639;
  double f640;
  double f641;
  double f642;
  double f643;
  double f644;
  double f645;
  double f646;
  double f647;
  double f648;
  double f649;
  double f650;
  double f651;
  double f652;
  double f653;
  double f654;
  double f655;
  double f656;
  double f657;
  double f658;
  double f659;
  double f660;
  double f661;
  double f662;
  double f663;
  double f664;
  double f665;
  double f666;
  double f667;
  double f668;
  double f669;
  double f670;
  double f671;
  double f672;
  double f673;
  double f674;
  double f675;
  double f676;
  double f677;
} cell_15;

#endif                                 /* typedef_cell_15 */

#ifndef typedef_cell_16
#define typedef_cell_16

typedef struct {
  double f1;
  double f2;
  double f3;
  double f4;
  double f5;
  double f6;
  double f7;
  double f8;
  double f9;
  double f10;
  double f11;
  double f12;
  double f13;
  double f14;
  double f15;
  double f16;
  double f17;
  double f18;
  double f19;
  double f20;
  double f21;
  double f22;
  double f23;
  double f24;
  double f25;
  double f26;
  double f27;
  double f28;
  double f29;
  double f30;
  double f31;
  double f32;
  double f33;
  double f34;
  double f35;
  double f36;
  double f37;
  double f38;
  double f39;
  double f40;
  double f41;
  double f42;
  double f43;
  double f44;
  double f45;
  double f46;
  double f47;
  double f48;
  double f49;
  double f50;
  double f51;
  double f52;
  double f53;
  double f54;
  double f55;
  double f56;
  double f57;
  double f58;
  double f59;
  double f60;
  double f61;
  double f62;
  double f63;
  double f64;
  double f65;
  double f66;
  double f67;
  double f68;
  double f69;
  double f70;
  double f71;
  double f72;
  double f73;
  double f74;
  double f75;
  double f76;
  double f77;
  double f78;
  double f79;
  double f80;
  double f81;
  double f82;
  double f83;
  double f84;
  double f85;
  double f86;
  double f87;
  double f88;
  double f89;
  double f90;
  double f91;
  double f92;
  double f93;
  double f94;
  double f95;
  double f96;
  double f97;
  double f98;
  double f99;
  double f100;
  double f101;
  double f102;
  double f103;
  double f104;
  double f105;
  double f106;
  double f107;
  double f108;
  double f109;
  double f110;
  double f111;
  double f112;
  double f113;
  double f114;
  double f115;
  double f116;
  double f117;
  double f118;
  double f119;
  double f120;
  double f121;
  double f122;
  double f123;
  double f124;
  double f125;
  double f126;
  double f127;
  double f128;
  double f129;
  double f130;
  double f131;
  double f132;
  double f133;
  double f134;
  double f135;
  double f136;
  double f137;
  double f138;
  double f139;
  double f140;
  double f141;
  double f142;
  double f143;
  double f144;
  double f145;
  double f146;
  double f147;
  double f148;
  double f149;
  double f150;
  double f151;
  double f152;
  double f153;
  double f154;
  double f155;
  double f156;
  double f157;
  double f158;
  double f159;
  double f160;
  double f161;
  double f162;
  double f163;
  double f164;
  double f165;
  double f166;
  double f167;
  double f168;
  double f169;
  double f170;
  double f171;
  double f172;
  double f173;
  double f174;
  double f175;
  double f176;
  double f177;
  double f178;
  double f179;
  double f180;
  double f181;
  double f182;
  double f183;
  double f184;
  double f185;
  double f186;
  double f187;
  double f188;
  double f189;
  double f190;
  double f191;
  double f192;
  double f193;
  double f194;
  double f195;
  double f196;
  double f197;
  double f198;
  double f199;
  double f200;
  double f201;
  double f202;
  double f203;
  double f204;
  double f205;
  double f206;
  double f207;
  double f208;
  double f209;
  double f210;
  double f211;
  double f212;
  double f213;
  double f214;
  double f215;
  double f216;
  double f217;
  double f218;
  double f219;
  double f220;
  double f221;
  double f222;
  double f223;
  double f224;
  double f225;
  double f226;
  double f227;
  double f228;
  double f229;
  double f230;
  creal_T f231;
  creal_T f232;
  creal_T f233;
  creal_T f234;
  creal_T f235;
  creal_T f236;
  creal_T f237;
  creal_T f238;
  creal_T f239;
  creal_T f240;
  creal_T f241;
  creal_T f242;
  creal_T f243;
  creal_T f244;
  creal_T f245;
  creal_T f246;
  creal_T f247;
  creal_T f248;
  creal_T f249;
  creal_T f250;
  creal_T f251;
  creal_T f252;
  creal_T f253;
  creal_T f254;
  creal_T f255;
  creal_T f256;
  creal_T f257;
  creal_T f258;
  creal_T f259;
  creal_T f260;
  double f261;
  creal_T f262;
  creal_T f263;
  creal_T f264;
  creal_T f265;
  creal_T f266;
  creal_T f267;
  creal_T f268;
  creal_T f269;
  creal_T f270;
  creal_T f271;
  creal_T f272;
  creal_T f273;
  creal_T f274;
  creal_T f275;
  creal_T f276;
  creal_T f277;
  creal_T f278;
  creal_T f279;
  creal_T f280;
  creal_T f281;
  creal_T f282;
  creal_T f283;
  creal_T f284;
  creal_T f285;
  creal_T f286;
  creal_T f287;
  creal_T f288;
  creal_T f289;
  creal_T f290;
  creal_T f291;
  creal_T f292;
  creal_T f293;
  creal_T f294;
  creal_T f295;
  creal_T f296;
  creal_T f297;
  creal_T f298;
  creal_T f299;
  creal_T f300;
  creal_T f301;
  creal_T f302;
  creal_T f303;
  creal_T f304;
  creal_T f305;
  creal_T f306;
  creal_T f307;
  creal_T f308;
  creal_T f309;
  creal_T f310;
  creal_T f311;
  creal_T f312;
  creal_T f313;
  creal_T f314;
  creal_T f315;
  creal_T f316;
  creal_T f317;
  creal_T f318;
  creal_T f319;
  creal_T f320;
  creal_T f321;
  creal_T f322;
  creal_T f323;
  creal_T f324;
  creal_T f325;
  double f326;
  double f327;
  creal_T f328;
  creal_T f329;
  creal_T f330;
  creal_T f331;
  creal_T f332;
  creal_T f333;
  creal_T f334;
  creal_T f335;
  creal_T f336;
  creal_T f337;
  creal_T f338;
  creal_T f339;
  creal_T f340;
  creal_T f341;
  creal_T f342;
  creal_T f343;
  creal_T f344;
  creal_T f345;
  creal_T f346;
  creal_T f347;
  creal_T f348;
  creal_T f349;
  creal_T f350;
  creal_T f351;
  creal_T f352;
  creal_T f353;
  creal_T f354;
  creal_T f355;
  creal_T f356;
  creal_T f357;
  creal_T f358;
  creal_T f359;
  creal_T f360;
  creal_T f361;
  creal_T f362;
  creal_T f363;
  creal_T f364;
  creal_T f365;
  creal_T f366;
  creal_T f367;
  creal_T f368;
  creal_T f369;
  creal_T f370;
  creal_T f371;
  creal_T f372;
  creal_T f373;
  creal_T f374;
  creal_T f375;
  creal_T f376;
  creal_T f377;
  creal_T f378;
  creal_T f379;
  creal_T f380;
  creal_T f381;
  creal_T f382;
  creal_T f383;
  creal_T f384;
  creal_T f385;
  creal_T f386;
  creal_T f387;
  creal_T f388;
  creal_T f389;
  creal_T f390;
  creal_T f391;
  creal_T f392;
  creal_T f393;
  creal_T f394;
  creal_T f395;
  creal_T f396;
  creal_T f397;
  creal_T f398;
  creal_T f399;
  creal_T f400;
  creal_T f401;
  creal_T f402;
  creal_T f403;
  creal_T f404;
  creal_T f405;
  creal_T f406;
  creal_T f407;
  creal_T f408;
  creal_T f409;
  creal_T f410;
  creal_T f411;
  creal_T f412;
  creal_T f413;
  creal_T f414;
  creal_T f415;
  creal_T f416;
  creal_T f417;
  creal_T f418;
  creal_T f419;
  creal_T f420;
  creal_T f421;
  creal_T f422;
  creal_T f423;
  creal_T f424;
  creal_T f425;
  creal_T f426;
  creal_T f427;
  double f428;
  double f429;
  creal_T f430;
  creal_T f431;
  creal_T f432;
  creal_T f433;
  creal_T f434;
  creal_T f435;
  creal_T f436;
  creal_T f437;
  creal_T f438;
  creal_T f439;
  double f440;
  creal_T f441;
  creal_T f442;
  creal_T f443;
  creal_T f444;
  creal_T f445;
  creal_T f446;
  creal_T f447;
  creal_T f448;
  creal_T f449;
  creal_T f450;
  double f451;
  creal_T f452;
  creal_T f453;
  creal_T f454;
  creal_T f455;
  creal_T f456;
  creal_T f457;
  creal_T f458;
  creal_T f459;
  creal_T f460;
  creal_T f461;
  double f462;
  creal_T f463;
  creal_T f464;
  creal_T f465;
  creal_T f466;
  creal_T f467;
  creal_T f468;
  creal_T f469;
  creal_T f470;
  creal_T f471;
  creal_T f472;
  double f473;
  creal_T f474;
  creal_T f475;
  creal_T f476;
  creal_T f477;
  creal_T f478;
  creal_T f479;
  creal_T f480;
  creal_T f481;
  creal_T f482;
  creal_T f483;
  double f484;
  creal_T f485;
  creal_T f486;
  creal_T f487;
  creal_T f488;
  creal_T f489;
  creal_T f490;
  creal_T f491;
  creal_T f492;
  creal_T f493;
  double f494;
  double f495;
  double f496;
  double f497;
  double f498;
  double f499;
  double f500;
  double f501;
  double f502;
  double f503;
  double f504;
  double f505;
  double f506;
  double f507;
  double f508;
  double f509;
  double f510;
  double f511;
  double f512;
  double f513;
  double f514;
  double f515;
  double f516;
  double f517;
  double f518;
  double f519;
  double f520;
  double f521;
  double f522;
  double f523;
  double f524;
  double f525;
  double f526;
  double f527;
  double f528;
  double f529;
  double f530;
  double f531;
  double f532;
  double f533;
  double f534;
  double f535;
  double f536;
  double f537;
  double f538;
  double f539;
  double f540;
  double f541;
  double f542;
  double f543;
  double f544;
  double f545;
  double f546;
  double f547;
  double f548;
  double f549;
  double f550;
  double f551;
  double f552;
  double f553;
  double f554;
  double f555;
  double f556;
  double f557;
  double f558;
  double f559;
  double f560;
  double f561;
  double f562;
  double f563;
  double f564;
  double f565;
  double f566;
  double f567;
  double f568;
  double f569;
  double f570;
  double f571;
  double f572;
  double f573;
  double f574;
  double f575;
  double f576;
  double f577;
  double f578;
  double f579;
  double f580;
  double f581;
  double f582;
  double f583;
  double f584;
  double f585;
  double f586;
  double f587;
  double f588;
  double f589;
  double f590;
  double f591;
  double f592;
  double f593;
  double f594;
  double f595;
  double f596;
  double f597;
  double f598;
  double f599;
  double f600;
  double f601;
  double f602;
  double f603;
  double f604;
  double f605;
  double f606;
  double f607;
  double f608;
  double f609;
  double f610;
  double f611;
  double f612;
  double f613;
  double f614;
  double f615;
  double f616;
  double f617;
  double f618;
  double f619;
  double f620;
  double f621;
  double f622;
  double f623;
  double f624;
  double f625;
  double f626;
  double f627;
  double f628;
  double f629;
  double f630;
  double f631;
  double f632;
  double f633;
  double f634;
  double f635;
  double f636;
  double f637;
  double f638;
  double f639;
  double f640;
  double f641;
  double f642;
  double f643;
  double f644;
  double f645;
  double f646;
  double f647;
  double f648;
  double f649;
  double f650;
  double f651;
  double f652;
  double f653;
  double f654;
  double f655;
  double f656;
  double f657;
  double f658;
  double f659;
  double f660;
  double f661;
  double f662;
  double f663;
  double f664;
  double f665;
  double f666;
  double f667;
  double f668;
  double f669;
  double f670;
  double f671;
  double f672;
  double f673;
  double f674;
  double f675;
  double f676;
  double f677;
  double f678;
  double f679;
  double f680;
  double f681;
  double f682;
  creal_T f683;
  creal_T f684;
  creal_T f685;
  creal_T f686;
  creal_T f687;
  creal_T f688;
  creal_T f689;
  double f690;
  creal_T f691;
  creal_T f692;
  double f693;
  creal_T f694;
  creal_T f695;
  creal_T f696;
  double f697;
  creal_T f698;
  creal_T f699;
  creal_T f700;
  creal_T f701;
  creal_T f702;
  creal_T f703;
  creal_T f704;
  creal_T f705;
  creal_T f706;
  creal_T f707;
  creal_T f708;
  creal_T f709;
  creal_T f710;
  creal_T f711;
  creal_T f712;
  creal_T f713;
  creal_T f714;
  creal_T f715;
  creal_T f716;
  creal_T f717;
  creal_T f718;
  creal_T f719;
  creal_T f720;
  creal_T f721;
  creal_T f722;
  creal_T f723;
  creal_T f724;
  creal_T f725;
  creal_T f726;
  creal_T f727;
  creal_T f728;
  creal_T f729;
  creal_T f730;
  creal_T f731;
  creal_T f732;
  creal_T f733;
  creal_T f734;
  creal_T f735;
  creal_T f736;
  creal_T f737;
  creal_T f738;
  creal_T f739;
  creal_T f740;
  creal_T f741;
  creal_T f742;
  creal_T f743;
  creal_T f744;
  creal_T f745;
  creal_T f746;
  creal_T f747;
  creal_T f748;
  creal_T f749;
  creal_T f750;
  creal_T f751;
  creal_T f752;
  creal_T f753;
  creal_T f754;
  creal_T f755;
  creal_T f756;
  creal_T f757;
  double f758;
  double f759;
  double f760;
  double f761;
  double f762;
  double f763;
  double f764;
  double f765;
  double f766;
  double f767;
  double f768;
  double f769;
  double f770;
  double f771;
  double f772;
  double f773;
  double f774;
  double f775;
  double f776;
  double f777;
  double f778;
  double f779;
  double f780;
  double f781;
  double f782;
  double f783;
  double f784;
  double f785;
  double f786;
  double f787;
  double f788;
  double f789;
  double f790;
  double f791;
  double f792;
  double f793;
  double f794;
  double f795;
  double f796;
  double f797;
  double f798;
  double f799;
  double f800;
  double f801;
  double f802;
  double f803;
  double f804;
  double f805;
  double f806;
  double f807;
  double f808;
  double f809;
  double f810;
  double f811;
  double f812;
  double f813;
  double f814;
  double f815;
  double f816;
  double f817;
  double f818;
  double f819;
  double f820;
  double f821;
  double f822;
  double f823;
  double f824;
  double f825;
  double f826;
  double f827;
  double f828;
  double f829;
  double f830;
  double f831;
  double f832;
  double f833;
  double f834;
  double f835;
  double f836;
  double f837;
  double f838;
  double f839;
  double f840;
  double f841;
  double f842;
  double f843;
  double f844;
  double f845;
  double f846;
  double f847;
  double f848;
  double f849;
  double f850;
  double f851;
  double f852;
  double f853;
  double f854;
  double f855;
  double f856;
  double f857;
  double f858;
  double f859;
  double f860;
  double f861;
  double f862;
  double f863;
  double f864;
  double f865;
  double f866;
  double f867;
  double f868;
  double f869;
  double f870;
  double f871;
  double f872;
  double f873;
  double f874;
  double f875;
  double f876;
  double f877;
  double f878;
  double f879;
  double f880;
  double f881;
  double f882;
  double f883;
  double f884;
  double f885;
  double f886;
  double f887;
  double f888;
  double f889;
  double f890;
  double f891;
  double f892;
  double f893;
  double f894;
  double f895;
  double f896;
  double f897;
  double f898;
  double f899;
  double f900;
  double f901;
  double f902;
  double f903;
  double f904;
  double f905;
  double f906;
  double f907;
  double f908;
  double f909;
  double f910;
  double f911;
  double f912;
  double f913;
  double f914;
  double f915;
  double f916;
  double f917;
  double f918;
  double f919;
  double f920;
  double f921;
  double f922;
  double f923;
  double f924;
  double f925;
  double f926;
  double f927;
  double f928;
  double f929;
  double f930;
  double f931;
  double f932;
  double f933;
  double f934;
  double f935;
  double f936;
  double f937;
  double f938;
  double f939;
  double f940;
  double f941;
  double f942;
  double f943;
  double f944;
  double f945;
  double f946;
  double f947;
  double f948;
  double f949;
  double f950;
  double f951;
  double f952;
  double f953;
  double f954;
  double f955;
} cell_16;

#endif                                 /* typedef_cell_16 */

/* Function Declarations */
static void ft_1(const cell_14 *ct, double c_data[], int c_size[2], double
                 ceq_data[], int ceq_size[2], creal_T c_gradient_data[], int
                 c_gradient_size[2], double ceq_gradient_data[], int
                 ceq_gradient_size[2]);
static void ft_2(const cell_15 *ct, double c_data[], int c_size[2], double
                 ceq_data[], int ceq_size[2], creal_T c_gradient_data[], int
                 c_gradient_size[2], double ceq_gradient_data[], int
                 ceq_gradient_size[2]);
static void ft_3(const cell_16 *ct, creal_T c_gradient[2304], double
                 ceq_gradient_data[], int ceq_gradient_size[2]);
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : const cell_14 *ct
 *                double c_data[]
 *                int c_size[2]
 *                double ceq_data[]
 *                int ceq_size[2]
 *                creal_T c_gradient_data[]
 *                int c_gradient_size[2]
 *                double ceq_gradient_data[]
 *                int ceq_gradient_size[2]
 * Return Type  : void
 */
static void ft_1(const cell_14 *ct, double c_data[], int c_size[2], double
                 ceq_data[], int ceq_size[2], creal_T c_gradient_data[], int
                 c_gradient_size[2], double ceq_gradient_data[], int
                 ceq_gradient_size[2])
{
  cell_15 expl_temp;
  creal_T b_c_gradient_data[2304];
  creal_T t1_f218;
  double b_ceq_gradient_data[324];
  double b_ceq_data[18];
  double ab_expl_temp_tmp;
  double b_ct_im;
  double b_ct_re;
  double b_expl_temp_tmp;
  double b_t626_im;
  double b_t626_re;
  double bb_expl_temp_tmp;
  double bim;
  double brm;
  double c_ct_im;
  double c_ct_re;
  double c_expl_temp_tmp;
  double cb_expl_temp_tmp;
  double ct_im;
  double ct_re;
  double d_expl_temp_tmp;
  double db_expl_temp_tmp;
  double e_expl_temp_tmp;
  double eb_expl_temp_tmp;
  double expl_temp_tmp;
  double f_expl_temp_tmp;
  double fb_expl_temp_tmp;
  double g_expl_temp_tmp;
  double gb_expl_temp_tmp;
  double h_expl_temp_tmp;
  double hb_expl_temp_tmp;
  double i_expl_temp_tmp;
  double j_expl_temp_tmp;
  double k_expl_temp_tmp;
  double l_expl_temp_tmp;
  double m_expl_temp_tmp;
  double n_expl_temp_tmp;
  double o_expl_temp_tmp;
  double p_expl_temp_tmp;
  double q_expl_temp_tmp;
  double r_expl_temp_tmp;
  double s;
  double s_expl_temp_tmp;
  double t1_f169_im;
  double t1_f169_re;
  double t1_f170_im;
  double t1_f170_re;
  double t1_f171_im;
  double t1_f171_re;
  double t1_f172_im;
  double t1_f172_re;
  double t1_f173_im;
  double t1_f173_re;
  double t1_f174_im;
  double t1_f174_re;
  double t1_f175_im;
  double t1_f175_re;
  double t1_f176_im;
  double t1_f176_re;
  double t1_f183_im;
  double t1_f183_re;
  double t1_f203_im;
  double t1_f203_re;
  double t1_f204_im;
  double t1_f204_re;
  double t514;
  double t515;
  double t516;
  double t517;
  double t518;
  double t519;
  double t520;
  double t521;
  double t522;
  double t523;
  double t524;
  double t525;
  double t526;
  double t527;
  double t528;
  double t529;
  double t530;
  double t531;
  double t532;
  double t533;
  double t534;
  double t535;
  double t536;
  double t537;
  double t538;
  double t539;
  double t540;
  double t541;
  double t542;
  double t543;
  double t544;
  double t545;
  double t546;
  double t547;
  double t548;
  double t549;
  double t554;
  double t555;
  double t556;
  double t557;
  double t558;
  double t559;
  double t560;
  double t561;
  double t562;
  double t563;
  double t564;
  double t565;
  double t566;
  double t567;
  double t568_im;
  double t568_re;
  double t569_im;
  double t569_re;
  double t570_im;
  double t570_re;
  double t571_im;
  double t571_re;
  double t572_im;
  double t572_re;
  double t573_im;
  double t573_re;
  double t574_im;
  double t574_re;
  double t575_im;
  double t575_re;
  double t576_im;
  double t576_re;
  double t595;
  double t597;
  double t599;
  double t601;
  double t603;
  double t605;
  double t607;
  double t609;
  double t611;
  double t622_im;
  double t622_re;
  double t623_im;
  double t623_re;
  double t624_im;
  double t624_re;
  double t625_im;
  double t625_re;
  double t626_im;
  double t626_re;
  double t627_im;
  double t627_re;
  double t628_im;
  double t628_re;
  double t629_im;
  double t629_re;
  double t630_im;
  double t630_re;
  double t_expl_temp_tmp;
  double u_expl_temp_tmp;
  double v_expl_temp_tmp;
  double w_expl_temp_tmp;
  double x_expl_temp_tmp;
  double y_expl_temp_tmp;
  int b_c_gradient_size[2];
  int b_ceq_gradient_size[2];
  int b_ceq_size[2];
  t554 = (((ct->f424 + ct->f470) + ct->f192) + ct->f292) + ct->f294;
  t555 = (((ct->f424 + ct->f473) + ct->f199) + ct->f293) + ct->f298;
  t556 = (((ct->f424 + ct->f477) + ct->f204) + ct->f295) + ct->f301;
  t557 = (((ct->f424 + ct->f480) + ct->f210) + ct->f299) + ct->f304;
  t558 = (((ct->f464 + ct->f478) + ct->f207) + ct->f296) + ct->f302;
  t559 = (((ct->f464 + ct->f481) + ct->f213) + ct->f300) + ct->f306;
  t560 = (((ct->f464 + ct->f56) + ct->f217) + ct->f303) + ct->f308;
  t561 = (((ct->f464 + ct->f58) + ct->f222) + ct->f307) + ct->f310;
  t562 = (((ct->f464 + ct->f60) + ct->f227) + ct->f309) + ct->f313;
  t563 = (((ct->f464 + ct->f64) + ct->f234) + ct->f312) + ct->f315;
  t564 = (((ct->f464 + ct->f67) + ct->f239) + ct->f314) + ct->f318;
  t565 = (((ct->f464 + ct->f69) + ct->f244) + ct->f316) + ct->f319;
  t566 = (((ct->f424 + ct->f86) + ct->f266) + ct->f340) + ct->f341;
  t567 = (((ct->f464 + ct->f88) + ct->f270) + ct->f342) + ct->f343;
  t514 = (((ct->f43 + ct->f462) + ct->f168) + ct->f170) + ct->f172;
  t515 = (((ct->f43 + ct->f463) + ct->f171) + ct->f173) + ct->f175;
  t516 = (((ct->f43 + ct->f465) + ct->f174) + ct->f176) + ct->f178;
  t517 = (((ct->f43 + ct->f466) + ct->f177) + ct->f180) + ct->f182;
  t518 = (((ct->f43 + ct->f467) + ct->f181) + ct->f183) + ct->f185;
  t519 = (((ct->f43 + ct->f469) + ct->f184) + ct->f186) + ct->f190;
  t520 = (((ct->f43 + ct->f471) + ct->f188) + ct->f191) + ct->f196;
  t521 = (((ct->f43 + ct->f474) + ct->f193) + ct->f197) + ct->f203;
  t522 = (((ct->f19 + ct->f472) + ct->f189) + ct->f194) + ct->f200;
  t523 = (((ct->f19 + ct->f475) + ct->f195) + ct->f201) + ct->f205;
  t524 = (((ct->f19 + ct->f479) + ct->f202) + ct->f206) + ct->f211;
  t525 = (((ct->f19 + ct->f482) + ct->f208) + ct->f212) + ct->f215;
  t526 = (((ct->f19 + ct->f57) + ct->f214) + ct->f216) + ct->f219;
  t527 = (((ct->f19 + ct->f59) + ct->f218) + ct->f221) + ct->f225;
  t528 = (((ct->f19 + ct->f61) + ct->f223) + ct->f226) + ct->f232;
  t529 = (((ct->f19 + ct->f65) + ct->f228) + ct->f233) + ct->f238;
  t530 = (((ct->f43 + ct->f85) + ct->f261) + ct->f262) + ct->f263;
  t531 = (((ct->f19 + ct->f87) + ct->f267) + ct->f268) + ct->f269;
  t532 = t514 * t514;
  t533 = t515 * t515;
  t534 = t516 * t516;
  t535 = t517 * t517;
  t536 = t518 * t518;
  t537 = t519 * t519;
  t538 = t520 * t520;
  t539 = t521 * t521;
  t540 = t522 * t522;
  t541 = t523 * t523;
  t542 = t524 * t524;
  t543 = t525 * t525;
  t544 = t526 * t526;
  t545 = t527 * t527;
  t546 = t528 * t528;
  t547 = t529 * t529;
  t548 = t530 * t530;
  t549 = t531 * t531;
  t568_re = ((((ct->f137.re + ct->f159.re) + ct->f347.re) + ct->f348.re) +
             ct->f350.re) + t514;
  t568_im = (((ct->f137.im + ct->f159.im) + ct->f347.im) + ct->f348.im) +
    ct->f350.im;
  t569_re = ((((ct->f137.re + ct->f160.re) + ct->f349.re) + ct->f351.re) +
             ct->f354.re) + t515;
  t569_im = (((ct->f137.im + ct->f160.im) + ct->f349.im) + ct->f351.im) +
    ct->f354.im;
  t570_re = ((((ct->f137.re + ct->f161.re) + ct->f352.re) + ct->f355.re) +
             ct->f357.re) + t516;
  t570_im = (((ct->f137.im + ct->f161.im) + ct->f352.im) + ct->f355.im) +
    ct->f357.im;
  t571_re = ((((ct->f137.re + ct->f162.re) + ct->f356.re) + ct->f358.re) +
             ct->f360.re) + t517;
  t571_im = (((ct->f137.im + ct->f162.im) + ct->f356.im) + ct->f358.im) +
    ct->f360.im;
  t572_re = ((((ct->f137.re + ct->f163.re) + ct->f359.re) + ct->f361.re) +
             ct->f363.re) + t518;
  t572_im = (((ct->f137.im + ct->f163.im) + ct->f359.im) + ct->f361.im) +
    ct->f363.im;
  t573_re = ((((ct->f137.re + ct->f164.re) + ct->f362.re) + ct->f365.re) +
             ct->f367.re) + t519;
  t573_im = (((ct->f137.im + ct->f164.im) + ct->f362.im) + ct->f365.im) +
    ct->f367.im;
  t574_re = ((((ct->f137.re + ct->f165.re) + ct->f366.re) + ct->f368.re) +
             ct->f370.re) + t520;
  t574_im = (((ct->f137.im + ct->f165.im) + ct->f366.im) + ct->f368.im) +
    ct->f370.im;
  t575_re = ((((ct->f137.re + ct->f166.re) + ct->f369.re) + ct->f371.re) +
             ct->f372.re) + t521;
  t575_im = (((ct->f137.im + ct->f166.im) + ct->f369.im) + ct->f371.im) +
    ct->f372.im;
  t576_re = ((((ct->f137.re + ct->f167.re) + ct->f373.re) + ct->f374.re) +
             ct->f376.re) + t530;
  t576_im = (((ct->f137.im + ct->f167.im) + ct->f373.im) + ct->f374.im) +
    ct->f376.im;
  t622_re = ((((ct->f278.re + ct->f279.re) + ct->f377.re) + ct->f378.re) +
             ct->f380.re) + t514;
  t622_im = (((ct->f278.im + ct->f279.im) + ct->f377.im) + ct->f378.im) +
    ct->f380.im;
  t623_re = ((((ct->f278.re + ct->f280.re) + ct->f379.re) + ct->f381.re) +
             ct->f383.re) + t515;
  t623_im = (((ct->f278.im + ct->f280.im) + ct->f379.im) + ct->f381.im) +
    ct->f383.im;
  t624_re = ((((ct->f278.re + ct->f281.re) + ct->f382.re) + ct->f384.re) +
             ct->f387.re) + t516;
  t624_im = (((ct->f278.im + ct->f281.im) + ct->f382.im) + ct->f384.im) +
    ct->f387.im;
  t625_re = ((((ct->f278.re + ct->f282.re) + ct->f385.re) + ct->f388.re) +
             ct->f390.re) + t517;
  t625_im = (((ct->f278.im + ct->f282.im) + ct->f385.im) + ct->f388.im) +
    ct->f390.im;
  t626_re = ((((ct->f278.re + ct->f283.re) + ct->f389.re) + ct->f391.re) +
             ct->f393.re) + t518;
  t626_im = (((ct->f278.im + ct->f283.im) + ct->f389.im) + ct->f391.im) +
    ct->f393.im;
  t627_re = ((((ct->f278.re + ct->f284.re) + ct->f392.re) + ct->f394.re) +
             ct->f396.re) + t519;
  t627_im = (((ct->f278.im + ct->f284.im) + ct->f392.im) + ct->f394.im) +
    ct->f396.im;
  t628_re = ((((ct->f278.re + ct->f285.re) + ct->f395.re) + ct->f398.re) +
             ct->f400.re) + t520;
  t628_im = (((ct->f278.im + ct->f285.im) + ct->f395.im) + ct->f398.im) +
    ct->f400.im;
  t629_re = ((((ct->f278.re + ct->f286.re) + ct->f399.re) + ct->f401.re) +
             ct->f402.re) + t521;
  t629_im = (((ct->f278.im + ct->f286.im) + ct->f399.im) + ct->f401.im) +
    ct->f402.im;
  t630_re = ((((ct->f278.re + ct->f288.re) + ct->f403.re) + ct->f404.re) +
             ct->f405.re) + t530;
  t630_im = (((ct->f278.im + ct->f288.im) + ct->f403.im) + ct->f404.im) +
    ct->f405.im;
  b_t626_re = ct->f487 * t626_re;
  b_t626_im = ct->f487 * t626_im;
  ct_re = ct->f375 * t626_re;
  ct_im = ct->f375 * t626_im;
  b_ct_re = ct->f407 * t626_re;
  b_ct_im = ct->f407 * t626_im;
  c_ct_re = ct->f425 * t626_re;
  c_ct_im = ct->f425 * t626_im;
  t595 = 1.0 / rt_hypotd_snf(t568_re, t568_im);
  t597 = 1.0 / rt_hypotd_snf(t569_re, t569_im);
  t599 = 1.0 / rt_hypotd_snf(t570_re, t570_im);
  t601 = 1.0 / rt_hypotd_snf(t571_re, t571_im);
  t603 = 1.0 / rt_hypotd_snf(t572_re, t572_im);
  t605 = 1.0 / rt_hypotd_snf(t573_re, t573_im);
  t607 = 1.0 / rt_hypotd_snf(t574_re, t574_im);
  t609 = 1.0 / rt_hypotd_snf(t575_re, t575_im);
  t611 = 1.0 / rt_hypotd_snf(t576_re, t576_im);
  t1_f218.re = t568_re * t622_re - t568_im * t622_im;
  t1_f218.im = t568_re * t622_im + t568_im * t622_re;
  b_sqrt(&t1_f218);
  if (t1_f218.im == 0.0) {
    t1_f169_re = 1.0 / t1_f218.re;
    t1_f169_im = 0.0;
  } else if (t1_f218.re == 0.0) {
    t1_f169_re = 0.0;
    t1_f169_im = -(1.0 / t1_f218.im);
  } else {
    brm = fabs(t1_f218.re);
    bim = fabs(t1_f218.im);
    if (brm > bim) {
      s = t1_f218.im / t1_f218.re;
      bim = t1_f218.re + s * t1_f218.im;
      t1_f169_re = (s * 0.0 + 1.0) / bim;
      t1_f169_im = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (t1_f218.re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }

      if (t1_f218.im > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }

      t1_f169_re = (bim + 0.0 * s) / brm;
      t1_f169_im = (0.0 * bim - s) / brm;
    } else {
      s = t1_f218.re / t1_f218.im;
      bim = t1_f218.im + s * t1_f218.re;
      t1_f169_re = s / bim;
      t1_f169_im = (s * 0.0 - 1.0) / bim;
    }
  }

  t1_f218.re = t569_re * t623_re - t569_im * t623_im;
  t1_f218.im = t569_re * t623_im + t569_im * t623_re;
  b_sqrt(&t1_f218);
  if (t1_f218.im == 0.0) {
    t1_f170_re = 1.0 / t1_f218.re;
    t1_f170_im = 0.0;
  } else if (t1_f218.re == 0.0) {
    t1_f170_re = 0.0;
    t1_f170_im = -(1.0 / t1_f218.im);
  } else {
    brm = fabs(t1_f218.re);
    bim = fabs(t1_f218.im);
    if (brm > bim) {
      s = t1_f218.im / t1_f218.re;
      bim = t1_f218.re + s * t1_f218.im;
      t1_f170_re = (s * 0.0 + 1.0) / bim;
      t1_f170_im = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (t1_f218.re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }

      if (t1_f218.im > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }

      t1_f170_re = (bim + 0.0 * s) / brm;
      t1_f170_im = (0.0 * bim - s) / brm;
    } else {
      s = t1_f218.re / t1_f218.im;
      bim = t1_f218.im + s * t1_f218.re;
      t1_f170_re = s / bim;
      t1_f170_im = (s * 0.0 - 1.0) / bim;
    }
  }

  t1_f218.re = t570_re * t624_re - t570_im * t624_im;
  t1_f218.im = t570_re * t624_im + t570_im * t624_re;
  b_sqrt(&t1_f218);
  if (t1_f218.im == 0.0) {
    t1_f171_re = 1.0 / t1_f218.re;
    t1_f171_im = 0.0;
  } else if (t1_f218.re == 0.0) {
    t1_f171_re = 0.0;
    t1_f171_im = -(1.0 / t1_f218.im);
  } else {
    brm = fabs(t1_f218.re);
    bim = fabs(t1_f218.im);
    if (brm > bim) {
      s = t1_f218.im / t1_f218.re;
      bim = t1_f218.re + s * t1_f218.im;
      t1_f171_re = (s * 0.0 + 1.0) / bim;
      t1_f171_im = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (t1_f218.re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }

      if (t1_f218.im > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }

      t1_f171_re = (bim + 0.0 * s) / brm;
      t1_f171_im = (0.0 * bim - s) / brm;
    } else {
      s = t1_f218.re / t1_f218.im;
      bim = t1_f218.im + s * t1_f218.re;
      t1_f171_re = s / bim;
      t1_f171_im = (s * 0.0 - 1.0) / bim;
    }
  }

  t1_f218.re = t571_re * t625_re - t571_im * t625_im;
  t1_f218.im = t571_re * t625_im + t571_im * t625_re;
  b_sqrt(&t1_f218);
  if (t1_f218.im == 0.0) {
    t1_f172_re = 1.0 / t1_f218.re;
    t1_f172_im = 0.0;
  } else if (t1_f218.re == 0.0) {
    t1_f172_re = 0.0;
    t1_f172_im = -(1.0 / t1_f218.im);
  } else {
    brm = fabs(t1_f218.re);
    bim = fabs(t1_f218.im);
    if (brm > bim) {
      s = t1_f218.im / t1_f218.re;
      bim = t1_f218.re + s * t1_f218.im;
      t1_f172_re = (s * 0.0 + 1.0) / bim;
      t1_f172_im = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (t1_f218.re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }

      if (t1_f218.im > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }

      t1_f172_re = (bim + 0.0 * s) / brm;
      t1_f172_im = (0.0 * bim - s) / brm;
    } else {
      s = t1_f218.re / t1_f218.im;
      bim = t1_f218.im + s * t1_f218.re;
      t1_f172_re = s / bim;
      t1_f172_im = (s * 0.0 - 1.0) / bim;
    }
  }

  t1_f218.re = t572_re * t626_re - t572_im * t626_im;
  t1_f218.im = t572_re * t626_im + t572_im * t626_re;
  b_sqrt(&t1_f218);
  if (t1_f218.im == 0.0) {
    t1_f173_re = 1.0 / t1_f218.re;
    t1_f173_im = 0.0;
  } else if (t1_f218.re == 0.0) {
    t1_f173_re = 0.0;
    t1_f173_im = -(1.0 / t1_f218.im);
  } else {
    brm = fabs(t1_f218.re);
    bim = fabs(t1_f218.im);
    if (brm > bim) {
      s = t1_f218.im / t1_f218.re;
      bim = t1_f218.re + s * t1_f218.im;
      t1_f173_re = (s * 0.0 + 1.0) / bim;
      t1_f173_im = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (t1_f218.re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }

      if (t1_f218.im > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }

      t1_f173_re = (bim + 0.0 * s) / brm;
      t1_f173_im = (0.0 * bim - s) / brm;
    } else {
      s = t1_f218.re / t1_f218.im;
      bim = t1_f218.im + s * t1_f218.re;
      t1_f173_re = s / bim;
      t1_f173_im = (s * 0.0 - 1.0) / bim;
    }
  }

  t1_f218.re = t573_re * t627_re - t573_im * t627_im;
  t1_f218.im = t573_re * t627_im + t573_im * t627_re;
  b_sqrt(&t1_f218);
  if (t1_f218.im == 0.0) {
    t1_f174_re = 1.0 / t1_f218.re;
    t1_f174_im = 0.0;
  } else if (t1_f218.re == 0.0) {
    t1_f174_re = 0.0;
    t1_f174_im = -(1.0 / t1_f218.im);
  } else {
    brm = fabs(t1_f218.re);
    bim = fabs(t1_f218.im);
    if (brm > bim) {
      s = t1_f218.im / t1_f218.re;
      bim = t1_f218.re + s * t1_f218.im;
      t1_f174_re = (s * 0.0 + 1.0) / bim;
      t1_f174_im = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (t1_f218.re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }

      if (t1_f218.im > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }

      t1_f174_re = (bim + 0.0 * s) / brm;
      t1_f174_im = (0.0 * bim - s) / brm;
    } else {
      s = t1_f218.re / t1_f218.im;
      bim = t1_f218.im + s * t1_f218.re;
      t1_f174_re = s / bim;
      t1_f174_im = (s * 0.0 - 1.0) / bim;
    }
  }

  t1_f218.re = t574_re * t628_re - t574_im * t628_im;
  t1_f218.im = t574_re * t628_im + t574_im * t628_re;
  b_sqrt(&t1_f218);
  if (t1_f218.im == 0.0) {
    t1_f175_re = 1.0 / t1_f218.re;
    t1_f175_im = 0.0;
  } else if (t1_f218.re == 0.0) {
    t1_f175_re = 0.0;
    t1_f175_im = -(1.0 / t1_f218.im);
  } else {
    brm = fabs(t1_f218.re);
    bim = fabs(t1_f218.im);
    if (brm > bim) {
      s = t1_f218.im / t1_f218.re;
      bim = t1_f218.re + s * t1_f218.im;
      t1_f175_re = (s * 0.0 + 1.0) / bim;
      t1_f175_im = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (t1_f218.re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }

      if (t1_f218.im > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }

      t1_f175_re = (bim + 0.0 * s) / brm;
      t1_f175_im = (0.0 * bim - s) / brm;
    } else {
      s = t1_f218.re / t1_f218.im;
      bim = t1_f218.im + s * t1_f218.re;
      t1_f175_re = s / bim;
      t1_f175_im = (s * 0.0 - 1.0) / bim;
    }
  }

  t1_f218.re = t575_re * t629_re - t575_im * t629_im;
  t1_f218.im = t575_re * t629_im + t575_im * t629_re;
  b_sqrt(&t1_f218);
  if (t1_f218.im == 0.0) {
    t1_f176_re = 1.0 / t1_f218.re;
    t1_f176_im = 0.0;
  } else if (t1_f218.re == 0.0) {
    t1_f176_re = 0.0;
    t1_f176_im = -(1.0 / t1_f218.im);
  } else {
    brm = fabs(t1_f218.re);
    bim = fabs(t1_f218.im);
    if (brm > bim) {
      s = t1_f218.im / t1_f218.re;
      bim = t1_f218.re + s * t1_f218.im;
      t1_f176_re = (s * 0.0 + 1.0) / bim;
      t1_f176_im = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (t1_f218.re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }

      if (t1_f218.im > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }

      t1_f176_re = (bim + 0.0 * s) / brm;
      t1_f176_im = (0.0 * bim - s) / brm;
    } else {
      s = t1_f218.re / t1_f218.im;
      bim = t1_f218.im + s * t1_f218.re;
      t1_f176_re = s / bim;
      t1_f176_im = (s * 0.0 - 1.0) / bim;
    }
  }

  t1_f218.re = t576_re * t630_re - t576_im * t630_im;
  t1_f218.im = t576_re * t630_im + t576_im * t630_re;
  b_sqrt(&t1_f218);
  t1_f183_re = ct->f135 * t572_re + ct->f135 * t626_re;
  t1_f183_im = ct->f135 * t572_im + ct->f135 * t626_im;
  t1_f203_re = ct->f111 * t572_re + ct->f111 * t626_re;
  t1_f203_im = ct->f111 * t572_im + ct->f111 * t626_im;
  t1_f204_re = ct->f117 * t572_re + ct->f117 * t626_re;
  t1_f204_im = ct->f117 * t572_im + ct->f117 * t626_im;
  t626_re = ct->f122 * t572_re + ct->f122 * t626_re;
  t626_im = ct->f122 * t572_im + ct->f122 * t626_im;
  expl_temp.f677 = ct->f491;
  expl_temp.f676 = ct->f490;
  expl_temp.f675 = ct->f489;
  expl_temp.f674 = ct->f488;
  expl_temp.f673 = ct->f487;
  expl_temp.f672 = ct->f486;
  expl_temp.f671 = ct->f485;
  expl_temp.f670 = ct->f484;
  expl_temp.f669 = ct->f483;
  expl_temp.f668 = ct->f265 * t531 * t611;
  bim = t531 * t611;
  expl_temp_tmp = bim * ct->f491;
  expl_temp.f667 = expl_temp_tmp * 4.0;
  expl_temp.f666 = ct->f265 * t530 * t611;
  s = t530 * t611;
  b_expl_temp_tmp = s * ct->f491;
  expl_temp.f665 = b_expl_temp_tmp * 4.0;
  expl_temp.f664 = ct->f242 * t529 * t609;
  brm = t529 * t609;
  c_expl_temp_tmp = brm * ct->f490;
  expl_temp.f663 = c_expl_temp_tmp * 4.0;
  expl_temp.f662 = ct->f231 * t528 * t607;
  d_expl_temp_tmp = t528 * t607;
  e_expl_temp_tmp = d_expl_temp_tmp * ct->f489;
  expl_temp.f661 = e_expl_temp_tmp * 4.0;
  expl_temp.f660 = ct->f220 * t527 * t605;
  f_expl_temp_tmp = t527 * t605;
  g_expl_temp_tmp = f_expl_temp_tmp * ct->f488;
  expl_temp.f659 = g_expl_temp_tmp * 4.0;
  expl_temp.f658 = ct->f242 * t521 * t609;
  h_expl_temp_tmp = t521 * t609;
  i_expl_temp_tmp = h_expl_temp_tmp * ct->f490;
  expl_temp.f657 = i_expl_temp_tmp * 4.0;
  expl_temp.f656 = ct->f209 * t526 * t603;
  j_expl_temp_tmp = t526 * t603;
  k_expl_temp_tmp = j_expl_temp_tmp * ct->f487;
  expl_temp.f655 = k_expl_temp_tmp * 4.0;
  expl_temp.f654 = ct->f231 * t520 * t607;
  l_expl_temp_tmp = t520 * t607;
  m_expl_temp_tmp = l_expl_temp_tmp * ct->f489;
  expl_temp.f653 = m_expl_temp_tmp * 4.0;
  expl_temp.f652 = ct->f187 * t525 * t601;
  n_expl_temp_tmp = t525 * t601;
  o_expl_temp_tmp = n_expl_temp_tmp * ct->f486;
  expl_temp.f651 = o_expl_temp_tmp * 4.0;
  expl_temp.f650 = ct->f220 * t519 * t605;
  p_expl_temp_tmp = t519 * t605;
  q_expl_temp_tmp = p_expl_temp_tmp * ct->f488;
  expl_temp.f649 = q_expl_temp_tmp * 4.0;
  expl_temp.f648 = ct->f169 * t524 * t599;
  r_expl_temp_tmp = t524 * t599;
  s_expl_temp_tmp = r_expl_temp_tmp * ct->f485;
  expl_temp.f647 = s_expl_temp_tmp * 4.0;
  expl_temp.f646 = ct->f209 * t518 * t603;
  t_expl_temp_tmp = t518 * t603;
  u_expl_temp_tmp = t_expl_temp_tmp * ct->f487;
  expl_temp.f645 = u_expl_temp_tmp * 4.0;
  expl_temp.f644 = ct->f146 * t523 * t597;
  v_expl_temp_tmp = t523 * t597;
  w_expl_temp_tmp = v_expl_temp_tmp * ct->f484;
  expl_temp.f643 = w_expl_temp_tmp * 4.0;
  expl_temp.f642 = ct->f187 * t517 * t601;
  x_expl_temp_tmp = t517 * t601;
  y_expl_temp_tmp = x_expl_temp_tmp * ct->f486;
  expl_temp.f641 = y_expl_temp_tmp * 4.0;
  expl_temp.f640 = ct->f124 * t522 * t595;
  ab_expl_temp_tmp = t522 * t595;
  bb_expl_temp_tmp = ab_expl_temp_tmp * ct->f483;
  expl_temp.f639 = bb_expl_temp_tmp * 4.0;
  expl_temp.f638 = ct->f169 * t516 * t599;
  cb_expl_temp_tmp = t516 * t599;
  db_expl_temp_tmp = cb_expl_temp_tmp * ct->f485;
  expl_temp.f637 = db_expl_temp_tmp * 4.0;
  expl_temp.f636 = ct->f146 * t515 * t597;
  eb_expl_temp_tmp = t515 * t597;
  fb_expl_temp_tmp = eb_expl_temp_tmp * ct->f484;
  expl_temp.f635 = fb_expl_temp_tmp * 4.0;
  expl_temp.f634 = ct->f124 * t514 * t595;
  gb_expl_temp_tmp = t514 * t595;
  hb_expl_temp_tmp = gb_expl_temp_tmp * ct->f483;
  expl_temp.f633 = hb_expl_temp_tmp * 4.0;
  expl_temp.f632 = t549 * t611;
  expl_temp.f631 = t548 * t611;
  expl_temp.f630 = t547 * t609;
  expl_temp.f629 = t546 * t607;
  expl_temp.f628 = t545 * t605;
  expl_temp.f627 = t539 * t609;
  expl_temp.f626 = t544 * t603;
  expl_temp.f625 = t538 * t607;
  expl_temp.f624 = t543 * t601;
  expl_temp.f623 = t537 * t605;
  expl_temp.f622 = t542 * t599;
  expl_temp.f621 = t536 * t603;
  expl_temp.f620 = t541 * t597;
  expl_temp.f619 = t535 * t601;
  expl_temp.f618 = t540 * t595;
  expl_temp.f617 = t534 * t599;
  expl_temp.f616 = t533 * t597;
  expl_temp.f615 = t532 * t595;
  expl_temp.f614 = ct->f476;
  expl_temp.f613 = bim * 2.0;
  expl_temp.f612 = s * 2.0;
  expl_temp.f611 = brm * 2.0;
  expl_temp.f610 = d_expl_temp_tmp * 2.0;
  expl_temp.f609 = f_expl_temp_tmp * 2.0;
  expl_temp.f608 = h_expl_temp_tmp * 2.0;
  expl_temp.f607 = j_expl_temp_tmp * 2.0;
  expl_temp.f606 = l_expl_temp_tmp * 2.0;
  expl_temp.f605 = n_expl_temp_tmp * 2.0;
  expl_temp.f604 = p_expl_temp_tmp * 2.0;
  expl_temp.f603 = r_expl_temp_tmp * 2.0;
  expl_temp.f602 = t_expl_temp_tmp * 2.0;
  expl_temp.f601 = v_expl_temp_tmp * 2.0;
  expl_temp.f600 = x_expl_temp_tmp * 2.0;
  expl_temp.f599 = ab_expl_temp_tmp * 2.0;
  expl_temp.f598 = cb_expl_temp_tmp * 2.0;
  expl_temp.f597 = eb_expl_temp_tmp * 2.0;
  expl_temp.f596 = gb_expl_temp_tmp * 2.0;
  expl_temp.f595 = ct->f440 * t611;
  expl_temp.f594 = ct->f439 * t611;
  expl_temp.f593 = ct->f438 * t609;
  expl_temp.f592 = ct->f437 * t607;
  expl_temp.f591 = ct->f435 * t605;
  expl_temp.f590 = ct->f429 * t609;
  expl_temp.f589 = ct->f434 * t603;
  expl_temp.f588 = ct->f428 * t607;
  expl_temp.f587 = ct->f433 * t601;
  expl_temp.f586 = ct->f427 * t605;
  expl_temp.f585 = ct->f432 * t599;
  expl_temp.f584 = ct->f468;
  expl_temp.f583 = ct->f426 * t603;
  expl_temp.f582 = ct->f431 * t597;
  expl_temp.f581 = ct->f423 * t601;
  expl_temp.f580 = ct->f430 * t595;
  expl_temp.f579 = ct->f422 * t599;
  expl_temp.f578 = ct->f421 * t597;
  expl_temp.f577 = ct->f420 * t595;
  bim = ct->f459 * t630_re;
  s = ct->f459 * t630_im;
  expl_temp.f576.re = bim * 0.0 - s * 5.0;
  expl_temp.f576.im = bim * 5.0 + s * 0.0;
  bim = ct->f458 * t630_re;
  s = ct->f458 * t630_im;
  expl_temp.f575.re = bim * 0.0 - s * 4.0;
  expl_temp.f575.im = bim * 4.0 + s * 0.0;
  bim = ct->f457 * t630_re;
  s = ct->f457 * t630_im;
  expl_temp.f574.re = bim * 0.0 - s * 3.0;
  expl_temp.f574.im = bim * 3.0 + s * 0.0;
  bim = ct->f455 * t629_re;
  s = ct->f455 * t629_im;
  expl_temp.f573.re = bim * 0.0 - s * 5.0;
  expl_temp.f573.im = bim * 5.0 + s * 0.0;
  bim = ct->f452 * t629_re;
  s = ct->f452 * t629_im;
  expl_temp.f572.re = bim * 0.0 - s * 4.0;
  expl_temp.f572.im = bim * 4.0 + s * 0.0;
  bim = ct->f449 * t629_re;
  s = ct->f449 * t629_im;
  expl_temp.f571.re = bim * 0.0 - s * 3.0;
  expl_temp.f571.im = bim * 3.0 + s * 0.0;
  bim = ct->f451 * t628_re;
  s = ct->f451 * t628_im;
  expl_temp.f570.re = bim * 0.0 - s * 5.0;
  expl_temp.f570.im = bim * 5.0 + s * 0.0;
  bim = ct->f444 * t628_re;
  s = ct->f444 * t628_im;
  expl_temp.f569.re = bim * 0.0 - s * 4.0;
  expl_temp.f569.im = bim * 4.0 + s * 0.0;
  bim = ct->f441 * t628_re;
  s = ct->f441 * t628_im;
  expl_temp.f568.re = bim * 0.0 - s * 3.0;
  expl_temp.f568.im = bim * 3.0 + s * 0.0;
  bim = ct->f443 * t627_re;
  s = ct->f443 * t627_im;
  expl_temp.f567.re = bim * 0.0 - s * 5.0;
  expl_temp.f567.im = bim * 5.0 + s * 0.0;
  bim = ct->f436 * t627_re;
  s = ct->f436 * t627_im;
  expl_temp.f566.re = bim * 0.0 - s * 4.0;
  expl_temp.f566.im = bim * 4.0 + s * 0.0;
  bim = ct->f417 * t627_re;
  s = ct->f417 * t627_im;
  expl_temp.f565.re = bim * 0.0 - s * 3.0;
  expl_temp.f565.im = bim * 3.0 + s * 0.0;
  expl_temp.f564.re = c_ct_re * 0.0 - c_ct_im * 5.0;
  expl_temp.f564.im = c_ct_re * 5.0 + c_ct_im * 0.0;
  expl_temp.f563.re = b_ct_re * 0.0 - b_ct_im * 4.0;
  expl_temp.f563.im = b_ct_re * 4.0 + b_ct_im * 0.0;
  expl_temp.f562.re = ct_re * 0.0 - ct_im * 3.0;
  expl_temp.f562.im = ct_re * 3.0 + ct_im * 0.0;
  ct_re = ct->f397 * t625_re;
  ct_im = ct->f397 * t625_im;
  expl_temp.f561.re = ct_re * 0.0 - ct_im * 5.0;
  expl_temp.f561.im = ct_re * 5.0 + ct_im * 0.0;
  ct_re = ct->f364 * t625_re;
  ct_im = ct->f364 * t625_im;
  expl_temp.f560.re = ct_re * 0.0 - ct_im * 4.0;
  expl_temp.f560.im = ct_re * 4.0 + ct_im * 0.0;
  ct_re = ct->f336 * t625_re;
  ct_im = ct->f336 * t625_im;
  expl_temp.f559.re = ct_re * 0.0 - ct_im * 3.0;
  expl_temp.f559.im = ct_re * 3.0 + ct_im * 0.0;
  ct_re = ct->f353 * t624_re;
  ct_im = ct->f353 * t624_im;
  expl_temp.f558.re = ct_re * 0.0 - ct_im * 5.0;
  expl_temp.f558.im = ct_re * 5.0 + ct_im * 0.0;
  ct_re = ct->f327 * t624_re;
  ct_im = ct->f327 * t624_im;
  expl_temp.f557.re = ct_re * 0.0 - ct_im * 4.0;
  expl_temp.f557.im = ct_re * 4.0 + ct_im * 0.0;
  ct_re = ct->f311 * t624_re;
  ct_im = ct->f311 * t624_im;
  expl_temp.f556.re = ct_re * 0.0 - ct_im * 3.0;
  expl_temp.f556.im = ct_re * 3.0 + ct_im * 0.0;
  ct_re = ct->f322 * t623_re;
  ct_im = ct->f322 * t623_im;
  expl_temp.f555.re = ct_re * 0.0 - ct_im * 5.0;
  expl_temp.f555.im = ct_re * 5.0 + ct_im * 0.0;
  ct_re = ct->f305 * t623_re;
  ct_im = ct->f305 * t623_im;
  expl_temp.f554.re = ct_re * 0.0 - ct_im * 4.0;
  expl_temp.f554.im = ct_re * 4.0 + ct_im * 0.0;
  ct_re = ct->f291 * t623_re;
  ct_im = ct->f291 * t623_im;
  expl_temp.f553.re = ct_re * 0.0 - ct_im * 3.0;
  expl_temp.f553.im = ct_re * 3.0 + ct_im * 0.0;
  ct_re = ct->f297 * t622_re;
  ct_im = ct->f297 * t622_im;
  expl_temp.f552.re = ct_re * 0.0 - ct_im * 5.0;
  expl_temp.f552.im = ct_re * 5.0 + ct_im * 0.0;
  ct_re = ct->f287 * t622_re;
  ct_im = ct->f287 * t622_im;
  expl_temp.f551.re = ct_re * 0.0 - ct_im * 4.0;
  expl_temp.f551.im = ct_re * 4.0 + ct_im * 0.0;
  ct_re = ct->f276 * t622_re;
  ct_im = ct->f276 * t622_im;
  expl_temp.f550.re = ct_re * 0.0 - ct_im * 3.0;
  expl_temp.f550.im = ct_re * 3.0 + ct_im * 0.0;
  bim = ct->f491 * t630_re;
  s = ct->f491 * t630_im;
  expl_temp.f549.re = bim * 0.0 - s * 2.0;
  expl_temp.f549.im = bim * 2.0 + s * 0.0;
  bim = ct->f490 * t629_re;
  s = ct->f490 * t629_im;
  expl_temp.f548.re = bim * 0.0 - s * 2.0;
  expl_temp.f548.im = bim * 2.0 + s * 0.0;
  bim = ct->f489 * t628_re;
  s = ct->f489 * t628_im;
  expl_temp.f547.re = bim * 0.0 - s * 2.0;
  expl_temp.f547.im = bim * 2.0 + s * 0.0;
  bim = ct->f488 * t627_re;
  s = ct->f488 * t627_im;
  expl_temp.f546.re = bim * 0.0 - s * 2.0;
  expl_temp.f546.im = bim * 2.0 + s * 0.0;
  expl_temp.f545.re = b_t626_re * 0.0 - b_t626_im * 2.0;
  expl_temp.f545.im = b_t626_re * 2.0 + b_t626_im * 0.0;
  bim = ct->f486 * t625_re;
  s = ct->f486 * t625_im;
  expl_temp.f544.re = bim * 0.0 - s * 2.0;
  expl_temp.f544.im = bim * 2.0 + s * 0.0;
  bim = ct->f485 * t624_re;
  s = ct->f485 * t624_im;
  expl_temp.f543.re = bim * 0.0 - s * 2.0;
  expl_temp.f543.im = bim * 2.0 + s * 0.0;
  bim = ct->f484 * t623_re;
  s = ct->f484 * t623_im;
  expl_temp.f542.re = bim * 0.0 - s * 2.0;
  expl_temp.f542.im = bim * 2.0 + s * 0.0;
  bim = ct->f483 * t622_re;
  s = ct->f483 * t622_im;
  expl_temp.f541.re = bim * 0.0 - s * 2.0;
  expl_temp.f541.im = bim * 2.0 + s * 0.0;
  ct_re = ct->f459 * t576_re;
  ct_im = ct->f459 * t576_im;
  expl_temp.f540.re = ct_re * 0.0 - ct_im * 5.0;
  expl_temp.f540.im = ct_re * 5.0 + ct_im * 0.0;
  ct_re = ct->f458 * t576_re;
  ct_im = ct->f458 * t576_im;
  expl_temp.f539.re = ct_re * 0.0 - ct_im * 4.0;
  expl_temp.f539.im = ct_re * 4.0 + ct_im * 0.0;
  ct_re = ct->f457 * t576_re;
  ct_im = ct->f457 * t576_im;
  expl_temp.f538.re = ct_re * 0.0 - ct_im * 3.0;
  expl_temp.f538.im = ct_re * 3.0 + ct_im * 0.0;
  ct_re = ct->f455 * t575_re;
  ct_im = ct->f455 * t575_im;
  expl_temp.f537.re = ct_re * 0.0 - ct_im * 5.0;
  expl_temp.f537.im = ct_re * 5.0 + ct_im * 0.0;
  ct_re = ct->f452 * t575_re;
  ct_im = ct->f452 * t575_im;
  expl_temp.f536.re = ct_re * 0.0 - ct_im * 4.0;
  expl_temp.f536.im = ct_re * 4.0 + ct_im * 0.0;
  ct_re = ct->f449 * t575_re;
  ct_im = ct->f449 * t575_im;
  expl_temp.f535.re = ct_re * 0.0 - ct_im * 3.0;
  expl_temp.f535.im = ct_re * 3.0 + ct_im * 0.0;
  ct_re = ct->f451 * t574_re;
  ct_im = ct->f451 * t574_im;
  expl_temp.f534.re = ct_re * 0.0 - ct_im * 5.0;
  expl_temp.f534.im = ct_re * 5.0 + ct_im * 0.0;
  ct_re = ct->f444 * t574_re;
  ct_im = ct->f444 * t574_im;
  expl_temp.f533.re = ct_re * 0.0 - ct_im * 4.0;
  expl_temp.f533.im = ct_re * 4.0 + ct_im * 0.0;
  ct_re = ct->f441 * t574_re;
  ct_im = ct->f441 * t574_im;
  expl_temp.f532.re = ct_re * 0.0 - ct_im * 3.0;
  expl_temp.f532.im = ct_re * 3.0 + ct_im * 0.0;
  ct_re = ct->f443 * t573_re;
  ct_im = ct->f443 * t573_im;
  expl_temp.f531.re = ct_re * 0.0 - ct_im * 5.0;
  expl_temp.f531.im = ct_re * 5.0 + ct_im * 0.0;
  ct_re = ct->f436 * t573_re;
  ct_im = ct->f436 * t573_im;
  expl_temp.f530.re = ct_re * 0.0 - ct_im * 4.0;
  expl_temp.f530.im = ct_re * 4.0 + ct_im * 0.0;
  ct_re = ct->f417 * t573_re;
  ct_im = ct->f417 * t573_im;
  expl_temp.f529.re = ct_re * 0.0 - ct_im * 3.0;
  expl_temp.f529.im = ct_re * 3.0 + ct_im * 0.0;
  ct_re = ct->f425 * t572_re;
  ct_im = ct->f425 * t572_im;
  expl_temp.f528.re = ct_re * 0.0 - ct_im * 5.0;
  expl_temp.f528.im = ct_re * 5.0 + ct_im * 0.0;
  ct_re = ct->f407 * t572_re;
  ct_im = ct->f407 * t572_im;
  expl_temp.f527.re = ct_re * 0.0 - ct_im * 4.0;
  expl_temp.f527.im = ct_re * 4.0 + ct_im * 0.0;
  ct_re = ct->f375 * t572_re;
  ct_im = ct->f375 * t572_im;
  expl_temp.f526.re = ct_re * 0.0 - ct_im * 3.0;
  expl_temp.f526.im = ct_re * 3.0 + ct_im * 0.0;
  ct_re = ct->f397 * t571_re;
  ct_im = ct->f397 * t571_im;
  expl_temp.f525.re = ct_re * 0.0 - ct_im * 5.0;
  expl_temp.f525.im = ct_re * 5.0 + ct_im * 0.0;
  ct_re = ct->f364 * t571_re;
  ct_im = ct->f364 * t571_im;
  expl_temp.f524.re = ct_re * 0.0 - ct_im * 4.0;
  expl_temp.f524.im = ct_re * 4.0 + ct_im * 0.0;
  ct_re = ct->f336 * t571_re;
  ct_im = ct->f336 * t571_im;
  expl_temp.f523.re = ct_re * 0.0 - ct_im * 3.0;
  expl_temp.f523.im = ct_re * 3.0 + ct_im * 0.0;
  ct_re = ct->f353 * t570_re;
  ct_im = ct->f353 * t570_im;
  expl_temp.f522.re = ct_re * 0.0 - ct_im * 5.0;
  expl_temp.f522.im = ct_re * 5.0 + ct_im * 0.0;
  ct_re = ct->f327 * t570_re;
  ct_im = ct->f327 * t570_im;
  expl_temp.f521.re = ct_re * 0.0 - ct_im * 4.0;
  expl_temp.f521.im = ct_re * 4.0 + ct_im * 0.0;
  ct_re = ct->f311 * t570_re;
  ct_im = ct->f311 * t570_im;
  expl_temp.f520.re = ct_re * 0.0 - ct_im * 3.0;
  expl_temp.f520.im = ct_re * 3.0 + ct_im * 0.0;
  ct_re = ct->f322 * t569_re;
  ct_im = ct->f322 * t569_im;
  expl_temp.f519.re = ct_re * 0.0 - ct_im * 5.0;
  expl_temp.f519.im = ct_re * 5.0 + ct_im * 0.0;
  ct_re = ct->f305 * t569_re;
  ct_im = ct->f305 * t569_im;
  expl_temp.f518.re = ct_re * 0.0 - ct_im * 4.0;
  expl_temp.f518.im = ct_re * 4.0 + ct_im * 0.0;
  ct_re = ct->f291 * t569_re;
  ct_im = ct->f291 * t569_im;
  expl_temp.f517.re = ct_re * 0.0 - ct_im * 3.0;
  expl_temp.f517.im = ct_re * 3.0 + ct_im * 0.0;
  expl_temp.f516 = ct->f461;
  ct_re = ct->f297 * t568_re;
  ct_im = ct->f297 * t568_im;
  expl_temp.f515.re = ct_re * 0.0 - ct_im * 5.0;
  expl_temp.f515.im = ct_re * 5.0 + ct_im * 0.0;
  ct_re = ct->f287 * t568_re;
  ct_im = ct->f287 * t568_im;
  expl_temp.f514.re = ct_re * 0.0 - ct_im * 4.0;
  expl_temp.f514.im = ct_re * 4.0 + ct_im * 0.0;
  ct_re = ct->f276 * t568_re;
  ct_im = ct->f276 * t568_im;
  expl_temp.f513.re = ct_re * 0.0 - ct_im * 3.0;
  expl_temp.f513.im = ct_re * 3.0 + ct_im * 0.0;
  expl_temp.f512 = ct->f460;
  expl_temp.f511 = ct->f459;
  expl_temp.f510 = ct->f458;
  expl_temp.f509 = ct->f457;
  bim = ct->f491 * t576_re;
  s = ct->f491 * t576_im;
  expl_temp.f508.re = bim * 0.0 - s * 2.0;
  expl_temp.f508.im = bim * 2.0 + s * 0.0;
  bim = ct->f490 * t575_re;
  s = ct->f490 * t575_im;
  expl_temp.f507.re = bim * 0.0 - s * 2.0;
  expl_temp.f507.im = bim * 2.0 + s * 0.0;
  expl_temp.f506 = ct->f456;
  bim = ct->f489 * t574_re;
  s = ct->f489 * t574_im;
  expl_temp.f505.re = bim * 0.0 - s * 2.0;
  expl_temp.f505.im = bim * 2.0 + s * 0.0;
  bim = ct->f488 * t573_re;
  s = ct->f488 * t573_im;
  expl_temp.f504.re = bim * 0.0 - s * 2.0;
  expl_temp.f504.im = bim * 2.0 + s * 0.0;
  t572_re *= ct->f487;
  t572_im *= ct->f487;
  expl_temp.f503.re = t572_re * 0.0 - t572_im * 2.0;
  expl_temp.f503.im = t572_re * 2.0 + t572_im * 0.0;
  bim = ct->f486 * t571_re;
  s = ct->f486 * t571_im;
  expl_temp.f502.re = bim * 0.0 - s * 2.0;
  expl_temp.f502.im = bim * 2.0 + s * 0.0;
  bim = ct->f485 * t570_re;
  s = ct->f485 * t570_im;
  expl_temp.f501.re = bim * 0.0 - s * 2.0;
  expl_temp.f501.im = bim * 2.0 + s * 0.0;
  bim = ct->f484 * t569_re;
  s = ct->f484 * t569_im;
  expl_temp.f500.re = bim * 0.0 - s * 2.0;
  expl_temp.f500.im = bim * 2.0 + s * 0.0;
  bim = ct->f483 * t568_re;
  s = ct->f483 * t568_im;
  expl_temp.f499.re = bim * 0.0 - s * 2.0;
  expl_temp.f499.im = bim * 2.0 + s * 0.0;
  expl_temp.f498 = t611 * t611;
  expl_temp.f497 = t611;
  expl_temp.f496 = t609 * t609;
  expl_temp.f495 = ct->f455;
  expl_temp.f494 = t609;
  expl_temp.f493 = t607 * t607;
  expl_temp.f492 = t607;
  expl_temp.f491 = t605 * t605;
  expl_temp.f490 = t605;
  expl_temp.f489 = t603 * t603;
  expl_temp.f488 = t603;
  expl_temp.f487 = t601 * t601;
  expl_temp.f486 = t601;
  expl_temp.f485 = t599 * t599;
  expl_temp.f484 = ct->f454;
  expl_temp.f483 = ct->f453;
  expl_temp.f482 = t599;
  expl_temp.f481 = t597 * t597;
  expl_temp.f480 = t597;
  expl_temp.f479 = t595 * t595;
  expl_temp.f478 = t595;
  expl_temp.f477 = ct->f452;
  expl_temp.f476 = ct->f451;
  expl_temp.f475 = ct->f450;
  expl_temp.f474 = t567;
  expl_temp.f473 = t566;
  expl_temp.f472 = t565;
  expl_temp.f471 = t564;
  expl_temp.f470 = t563;
  expl_temp.f469 = t562;
  expl_temp.f468 = t561;
  expl_temp.f467 = t560;
  expl_temp.f466 = ct->f449;
  expl_temp.f465 = t559;
  expl_temp.f464 = t558;
  expl_temp.f463 = t557;
  expl_temp.f462 = t556;
  expl_temp.f461 = t555;
  expl_temp.f460 = t554;
  expl_temp.f459 = ct->f448;
  expl_temp.f458 = ct->f447;
  expl_temp.f457 = ct->f446;
  expl_temp.f456 = ct->f445;
  expl_temp.f455 = ct->f444;
  expl_temp.f454 = t549;
  expl_temp.f453 = t548;
  expl_temp.f452 = t547;
  expl_temp.f451 = t546;
  expl_temp.f450 = t545;
  expl_temp.f449 = t544;
  expl_temp.f448 = t543;
  expl_temp.f447 = t542;
  expl_temp.f446 = t541;
  expl_temp.f445 = t540;
  expl_temp.f444 = ct->f443;
  expl_temp.f443 = t539;
  expl_temp.f442 = t538;
  expl_temp.f441 = t537;
  expl_temp.f440 = t536;
  expl_temp.f439 = t535;
  expl_temp.f438 = t534;
  expl_temp.f437 = t533;
  expl_temp.f436 = t532;
  expl_temp.f435 = t531;
  expl_temp.f434 = t530;
  expl_temp.f433 = ct->f442;
  expl_temp.f432 = t529;
  expl_temp.f431 = t528;
  expl_temp.f430 = t527;
  expl_temp.f429 = t526;
  expl_temp.f428 = t525;
  expl_temp.f427 = t524;
  expl_temp.f426 = t523;
  expl_temp.f425 = t522;
  expl_temp.f424 = t521;
  expl_temp.f423 = t520;
  expl_temp.f422 = ct->f441;
  expl_temp.f421 = t519;
  expl_temp.f420 = t518;
  expl_temp.f419 = t517;
  expl_temp.f418 = t516;
  expl_temp.f417 = t515;
  expl_temp.f416 = t514;
  expl_temp.f415 = ct->f440;
  expl_temp.f414 = ct->f439;
  expl_temp.f413 = ct->f438;
  expl_temp.f412 = ct->f437;
  expl_temp.f411 = ct->f436;
  expl_temp.f410 = ct->f435;
  expl_temp.f409 = ct->f434;
  expl_temp.f408 = ct->f433;
  expl_temp.f407 = ct->f432;
  expl_temp.f406 = ct->f431;
  expl_temp.f405 = ct->f430;
  expl_temp.f404 = ct->f429;
  expl_temp.f403 = ct->f428;
  expl_temp.f402 = ct->f427;
  expl_temp.f401 = ct->f426;
  expl_temp.f400 = ct->f425;
  expl_temp.f399 = ct->f423;
  expl_temp.f398 = ct->f422;
  expl_temp.f397 = ct->f421;
  expl_temp.f396 = ct->f420;
  bim = ct->f43 * ct->f17;
  expl_temp.f395 = bim * ct->f19 * ct->f406 * -2.0;
  expl_temp.f394 = ct->f41 * ct->f43 * ct->f19 * ct->f406 * -2.0;
  expl_temp.f393 = ct->f453 * ct->f63 * ct->f406;
  expl_temp.f392 = bim * ct->f424 * ct->f406;
  expl_temp.f391 = ct->f335 * ct->f63 * ct->f406;
  expl_temp.f390 = ct->f335 * ct->f476 * ct->f406;
  expl_temp.f389 = ct->f419;
  expl_temp.f388 = ct->f17 * ct->f19 * ct->f424 * ct->f406;
  expl_temp.f387 = ct->f43 * ct->f19 * ct->f335 * ct->f406;
  expl_temp.f386 = ct->f19 * ct->f476 * ct->f406;
  expl_temp.f385 = ct->f43 * ct->f63 * ct->f406;
  expl_temp.f384 = ct->f19 * ct->f410;
  expl_temp.f383 = ct->f17 * ct->f409;
  expl_temp.f382 = ct->f19 * ct->f408;
  expl_temp.f381 = ct->f43 * ct->f408;
  expl_temp.f380 = ct->f72 * ct->f406;
  expl_temp.f379 = ct->f418;
  expl_temp.f378 = ct->f417;
  expl_temp.f377 = ct->f55 * ct->f406;
  expl_temp.f376 = ct->f416;
  expl_temp.f375 = ct->f415;
  expl_temp.f374 = ct->f414;
  expl_temp.f373 = ct->f413;
  expl_temp.f372 = ct->f412;
  expl_temp.f371 = ct->f411;
  expl_temp.f370 = ct->f410;
  expl_temp.f369 = ct->f409;
  expl_temp.f368 = ct->f408;
  expl_temp.f367 = ct->f407;
  expl_temp.f366 = ct->f406;
  expl_temp.f365 = ct->f397;
  expl_temp.f364 = ct->f386;
  expl_temp.f363 = ct->f375;
  expl_temp.f362 = ct->f364;
  expl_temp.f361 = ct->f353;
  expl_temp.f360 = ct->f346;
  expl_temp.f359 = ct->f345;
  expl_temp.f358 = ct->f344;
  expl_temp.f357 = ct->f339;
  expl_temp.f356 = ct->f338;
  expl_temp.f355 = ct->f337;
  expl_temp.f354 = ct->f336;
  expl_temp.f353 = ct->f335;
  expl_temp.f352 = ct->f334;
  expl_temp.f351 = ct->f333;
  expl_temp.f350 = ct->f332;
  expl_temp.f349 = ct->f331;
  expl_temp.f348 = ct->f330;
  expl_temp.f347 = ct->f329;
  expl_temp.f346 = ct->f328;
  expl_temp.f345 = ct->f327;
  expl_temp.f344 = ct->f326;
  expl_temp.f343 = ct->f325;
  expl_temp.f342 = ct->f324;
  expl_temp.f341 = ct->f323;
  expl_temp.f340 = ct->f322;
  expl_temp.f339 = ct->f321;
  expl_temp.f338 = ct->f320;
  expl_temp.f337 = ct->f317;
  expl_temp.f336 = ct->f311;
  expl_temp.f335 = ct->f305;
  expl_temp.f334 = ct->f297;
  expl_temp.f333 = ct->f291;
  expl_temp.f332 = ct->f290;
  expl_temp.f331 = ct->f289;
  expl_temp.f330 = ct->f287;
  expl_temp.f329 = ct->f277;
  expl_temp.f328 = ct->f276;
  expl_temp.f327 = ct->f275;
  expl_temp.f326 = ct->f274;
  expl_temp.f325 = ct->f273;
  expl_temp.f324 = ct->f272;
  expl_temp.f323 = ct->f271;
  expl_temp.f322 = ct->f265;
  expl_temp.f321 = ct->f264;
  expl_temp.f320 = ct->f260;
  expl_temp.f319 = ct->f259;
  expl_temp.f318 = ct->f258;
  expl_temp.f317 = ct->f257;
  expl_temp.f316 = ct->f256;
  expl_temp.f315 = ct->f255;
  expl_temp.f314 = ct->f254;
  expl_temp.f313 = ct->f253;
  expl_temp.f312 = ct->f252;
  expl_temp.f311 = ct->f251;
  expl_temp.f310 = ct->f250;
  expl_temp.f309 = ct->f249;
  expl_temp.f308 = ct->f248;
  expl_temp.f307 = ct->f247;
  expl_temp.f306 = ct->f246;
  expl_temp.f305 = ct->f245;
  expl_temp.f304 = ct->f243;
  expl_temp.f303 = ct->f242;
  expl_temp.f302 = ct->f241;
  expl_temp.f301 = ct->f240;
  expl_temp.f300 = ct->f237;
  expl_temp.f299 = ct->f236;
  expl_temp.f298 = ct->f235;
  expl_temp.f297 = ct->f231;
  expl_temp.f296 = ct->f230;
  expl_temp.f295 = ct->f229;
  expl_temp.f294 = ct->f224;
  expl_temp.f293 = ct->f220;
  expl_temp.f292 = ct->f209;
  expl_temp.f291 = ct->f198;
  expl_temp.f290 = ct->f187;
  expl_temp.f289 = ct->f179;
  expl_temp.f288 = ct->f169;
  expl_temp.f287 = ct->f158;
  expl_temp.f286 = ct->f157;
  expl_temp.f285 = ct->f156;
  expl_temp.f284 = ct->f155;
  expl_temp.f283 = ct->f154;
  expl_temp.f282 = ct->f153;
  expl_temp.f281 = ct->f152;
  expl_temp.f280 = ct->f151;
  expl_temp.f279 = ct->f150;
  expl_temp.f278 = ct->f149;
  expl_temp.f277 = ct->f148;
  expl_temp.f276 = ct->f147;
  expl_temp.f275 = ct->f146;
  expl_temp.f274 = ct->f145;
  expl_temp.f273 = ct->f144;
  expl_temp.f272 = ct->f143;
  expl_temp.f271 = ct->f142;
  expl_temp.f270 = ct->f141;
  expl_temp.f269 = ct->f140;
  expl_temp.f268 = ct->f139;
  expl_temp.f267 = ct->f138;
  expl_temp.f266 = ct->f136;
  expl_temp.f265 = ct->f135;
  expl_temp.f264 = ct->f134;
  expl_temp.f263 = ct->f133;
  expl_temp.f262 = ct->f132;
  expl_temp.f261 = ct->f131;
  expl_temp.f260 = ct->f130;
  expl_temp.f259 = ct->f129;
  expl_temp.f258 = ct->f128;
  expl_temp.f257 = ct->f127;
  expl_temp.f256 = ct->f126;
  expl_temp.f255 = ct->f125;
  expl_temp.f254 = ct->f124;
  expl_temp.f253 = ct->f123;
  expl_temp.f252 = ct->f122;
  expl_temp.f251 = ct->f121;
  expl_temp.f250 = ct->f120;
  expl_temp.f249 = ct->f119;
  expl_temp.f248 = ct->f118;
  expl_temp.f247 = ct->f117;
  expl_temp.f246 = ct->f116;
  expl_temp.f245 = ct->f115;
  expl_temp.f244 = ct->f114;
  expl_temp.f243 = ct->f113;
  expl_temp.f242 = ct->f112;
  expl_temp.f241 = ct->f111;
  expl_temp.f240 = ct->f110;
  expl_temp.f239 = ct->f109;
  expl_temp.f238 = ct->f108;
  expl_temp.f237 = ct->f107;
  expl_temp.f236 = ct->f106;
  expl_temp.f235 = ct->f105;
  expl_temp.f234 = ct->f104;
  expl_temp.f233 = ct->f103;
  expl_temp.f232 = ct->f102;
  expl_temp.f231 = ct->f101;
  expl_temp.f230 = ct->f100;
  expl_temp.f229 = ct->f99;
  expl_temp.f228 = ct->f98;
  expl_temp.f227 = ct->f97;
  expl_temp.f226 = ct->f96;
  expl_temp.f225 = ct->f95;
  expl_temp.f224 = ct->f94;
  expl_temp.f223 = ct->f93;
  expl_temp.f222 = ct->f92;
  expl_temp.f221 = ct->f91;
  expl_temp.f220 = ct->f90;
  expl_temp.f219 = ct->f89;
  expl_temp.f218.re = ct->f131 * t576_re + ct->f131 * t630_re;
  expl_temp.f218.im = ct->f131 * t576_im + ct->f131 * t630_im;
  expl_temp.f217.re = ct->f130 * t576_re + ct->f130 * t630_re;
  expl_temp.f217.im = ct->f130 * t576_im + ct->f130 * t630_im;
  expl_temp.f216.re = ct->f129 * t576_re + ct->f129 * t630_re;
  expl_temp.f216.im = ct->f129 * t576_im + ct->f129 * t630_im;
  expl_temp.f215.re = ct->f128 * t575_re + ct->f128 * t629_re;
  expl_temp.f215.im = ct->f128 * t575_im + ct->f128 * t629_im;
  expl_temp.f214.re = ct->f126 * t575_re + ct->f126 * t629_re;
  expl_temp.f214.im = ct->f126 * t575_im + ct->f126 * t629_im;
  expl_temp.f213 = ct->f84;
  expl_temp.f212.re = ct->f121 * t575_re + ct->f121 * t629_re;
  expl_temp.f212.im = ct->f121 * t575_im + ct->f121 * t629_im;
  expl_temp.f211.re = ct->f127 * t574_re + ct->f127 * t628_re;
  expl_temp.f211.im = ct->f127 * t574_im + ct->f127 * t628_im;
  expl_temp.f210.re = ct->f123 * t574_re + ct->f123 * t628_re;
  expl_temp.f210.im = ct->f123 * t574_im + ct->f123 * t628_im;
  expl_temp.f209.re = ct->f118 * t574_re + ct->f118 * t628_re;
  expl_temp.f209.im = ct->f118 * t574_im + ct->f118 * t628_im;
  expl_temp.f208.re = ct->f125 * t573_re + ct->f125 * t627_re;
  expl_temp.f208.im = ct->f125 * t573_im + ct->f125 * t627_im;
  expl_temp.f207.re = ct->f120 * t573_re + ct->f120 * t627_re;
  expl_temp.f207.im = ct->f120 * t573_im + ct->f120 * t627_im;
  expl_temp.f206.re = ct->f115 * t573_re + ct->f115 * t627_re;
  expl_temp.f206.im = ct->f115 * t573_im + ct->f115 * t627_im;
  expl_temp.f205.re = t626_re;
  expl_temp.f205.im = t626_im;
  expl_temp.f204.re = t1_f204_re;
  expl_temp.f204.im = t1_f204_im;
  expl_temp.f203.re = t1_f203_re;
  expl_temp.f203.im = t1_f203_im;
  expl_temp.f202 = ct->f83;
  expl_temp.f201 = ct->f82;
  expl_temp.f200.re = ct->f119 * t571_re + ct->f119 * t625_re;
  expl_temp.f200.im = ct->f119 * t571_im + ct->f119 * t625_im;
  expl_temp.f199.re = ct->f114 * t571_re + ct->f114 * t625_re;
  expl_temp.f199.im = ct->f114 * t571_im + ct->f114 * t625_im;
  expl_temp.f198.re = ct->f108 * t571_re + ct->f108 * t625_re;
  expl_temp.f198.im = ct->f108 * t571_im + ct->f108 * t625_im;
  expl_temp.f197.re = ct->f116 * t570_re + ct->f116 * t624_re;
  expl_temp.f197.im = ct->f116 * t570_im + ct->f116 * t624_im;
  expl_temp.f196.re = ct->f110 * t570_re + ct->f110 * t624_re;
  expl_temp.f196.im = ct->f110 * t570_im + ct->f110 * t624_im;
  expl_temp.f195.re = ct->f106 * t570_re + ct->f106 * t624_re;
  expl_temp.f195.im = ct->f106 * t570_im + ct->f106 * t624_im;
  expl_temp.f194.re = ct->f112 * t569_re + ct->f112 * t623_re;
  expl_temp.f194.im = ct->f112 * t569_im + ct->f112 * t623_im;
  expl_temp.f193.re = ct->f107 * t569_re + ct->f107 * t623_re;
  expl_temp.f193.im = ct->f107 * t569_im + ct->f107 * t623_im;
  expl_temp.f192.re = ct->f104 * t569_re + ct->f104 * t623_re;
  expl_temp.f192.im = ct->f104 * t569_im + ct->f104 * t623_im;
  expl_temp.f191.re = ct->f109 * t568_re + ct->f109 * t622_re;
  expl_temp.f191.im = ct->f109 * t568_im + ct->f109 * t622_im;
  expl_temp.f190 = ct->f81;
  expl_temp.f189.re = ct->f105 * t568_re + ct->f105 * t622_re;
  expl_temp.f189.im = ct->f105 * t568_im + ct->f105 * t622_im;
  expl_temp.f188.re = ct->f103 * t568_re + ct->f103 * t622_re;
  expl_temp.f188.im = ct->f103 * t568_im + ct->f103 * t622_im;
  expl_temp.f187.re = ct->f253 * t576_re + ct->f253 * t630_re;
  expl_temp.f187.im = ct->f253 * t576_im + ct->f253 * t630_im;
  expl_temp.f186.re = ct->f198 * t575_re + ct->f198 * t629_re;
  expl_temp.f186.im = ct->f198 * t575_im + ct->f198 * t629_im;
  expl_temp.f185.re = ct->f179 * t574_re + ct->f179 * t628_re;
  expl_temp.f185.im = ct->f179 * t574_im + ct->f179 * t628_im;
  expl_temp.f184.re = ct->f158 * t573_re + ct->f158 * t627_re;
  expl_temp.f184.im = ct->f158 * t573_im + ct->f158 * t627_im;
  expl_temp.f183.re = t1_f183_re;
  expl_temp.f183.im = t1_f183_im;
  expl_temp.f182.re = ct->f113 * t571_re + ct->f113 * t625_re;
  expl_temp.f182.im = ct->f113 * t571_im + ct->f113 * t625_im;
  expl_temp.f181.re = ct->f102 * t570_re + ct->f102 * t624_re;
  expl_temp.f181.im = ct->f102 * t570_im + ct->f102 * t624_im;
  expl_temp.f180.re = ct->f91 * t569_re + ct->f91 * t623_re;
  expl_temp.f180.im = ct->f91 * t569_im + ct->f91 * t623_im;
  expl_temp.f179 = ct->f80;
  expl_temp.f178.re = ct->f82 * t568_re + ct->f82 * t622_re;
  expl_temp.f178.im = ct->f82 * t568_im + ct->f82 * t622_im;
  if (t1_f218.im == 0.0) {
    expl_temp.f177.re = 1.0 / t1_f218.re;
    expl_temp.f177.im = 0.0;
  } else if (t1_f218.re == 0.0) {
    expl_temp.f177.re = 0.0;
    expl_temp.f177.im = -(1.0 / t1_f218.im);
  } else {
    brm = fabs(t1_f218.re);
    bim = fabs(t1_f218.im);
    if (brm > bim) {
      s = t1_f218.im / t1_f218.re;
      bim = t1_f218.re + s * t1_f218.im;
      expl_temp.f177.re = (s * 0.0 + 1.0) / bim;
      expl_temp.f177.im = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (t1_f218.re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }

      if (t1_f218.im > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }

      expl_temp.f177.re = (bim + 0.0 * s) / brm;
      expl_temp.f177.im = (0.0 * bim - s) / brm;
    } else {
      s = t1_f218.re / t1_f218.im;
      bim = t1_f218.im + s * t1_f218.re;
      expl_temp.f177.re = s / bim;
      expl_temp.f177.im = (s * 0.0 - 1.0) / bim;
    }
  }

  expl_temp.f176.re = t1_f176_re;
  expl_temp.f176.im = t1_f176_im;
  expl_temp.f175.re = t1_f175_re;
  expl_temp.f175.im = t1_f175_im;
  expl_temp.f174.re = t1_f174_re;
  expl_temp.f174.im = t1_f174_im;
  expl_temp.f173.re = t1_f173_re;
  expl_temp.f173.im = t1_f173_im;
  expl_temp.f172.re = t1_f172_re;
  expl_temp.f172.im = t1_f172_im;
  expl_temp.f171.re = t1_f171_re;
  expl_temp.f171.im = t1_f171_im;
  expl_temp.f170.re = t1_f170_re;
  expl_temp.f170.im = t1_f170_im;
  expl_temp.f169.re = t1_f169_re;
  expl_temp.f169.im = t1_f169_im;
  expl_temp.f168 = ct->f79;
  expl_temp.f167 = ct->f78;
  expl_temp.f166 = ct->f77;
  expl_temp.f165 = ct->f76;
  expl_temp.f164 = ct->f75;
  expl_temp.f163 = ct->f74;
  expl_temp.f162 = ct->f73;
  expl_temp.f161 = ct->f71;
  expl_temp.f160 = ct->f458 * t530 * t611 * 8.0;
  expl_temp.f159 = ct->f154 * t529 * t609;
  expl_temp.f158 = ct->f70;
  expl_temp.f157 = ct->f455 * t529 * t609 * 10.0;
  expl_temp.f156 = ct->f145 * t529 * t609;
  expl_temp.f155 = ct->f452 * t529 * t609 * 8.0;
  expl_temp.f154 = ct->f153 * t528 * t607;
  expl_temp.f153 = ct->f451 * t528 * t607 * 10.0;
  expl_temp.f152 = ct->f144 * t528 * t607;
  expl_temp.f151 = ct->f444 * t528 * t607 * 8.0;
  expl_temp.f150 = ct->f152 * t527 * t605;
  expl_temp.f149 = ct->f154 * t521 * t609;
  expl_temp.f148 = ct->f443 * t527 * t605 * 10.0;
  expl_temp.f147 = ct->f143 * t527 * t605;
  expl_temp.f146 = ct->f436 * t527 * t605 * 8.0;
  expl_temp.f145 = ct->f455 * t521 * t609 * 10.0;
  expl_temp.f144 = ct->f145 * t521 * t609;
  expl_temp.f143 = ct->f151 * t526 * t603;
  expl_temp.f142 = ct->f452 * t521 * t609 * 8.0;
  expl_temp.f141 = ct->f153 * t520 * t607;
  expl_temp.f140 = ct->f425 * t526 * t603 * 10.0;
  expl_temp.f139 = ct->f142 * t526 * t603;
  expl_temp.f138 = ct->f407 * t526 * t603 * 8.0;
  expl_temp.f137 = ct->f451 * t520 * t607 * 10.0;
  expl_temp.f136 = ct->f144 * t520 * t607;
  expl_temp.f135 = ct->f150 * t525 * t601;
  expl_temp.f134 = ct->f444 * t520 * t607 * 8.0;
  expl_temp.f133 = ct->f152 * t519 * t605;
  expl_temp.f132 = ct->f397 * t525 * t601 * 10.0;
  expl_temp.f131 = ct->f141 * t525 * t601;
  expl_temp.f130 = ct->f364 * t525 * t601 * 8.0;
  expl_temp.f129 = ct->f443 * t519 * t605 * 10.0;
  expl_temp.f128 = ct->f143 * t519 * t605;
  expl_temp.f127 = ct->f68;
  expl_temp.f126 = ct->f149 * t524 * t599;
  expl_temp.f125 = ct->f436 * t519 * t605 * 8.0;
  expl_temp.f124 = ct->f151 * t518 * t603;
  expl_temp.f123 = ct->f353 * t524 * t599 * 10.0;
  expl_temp.f122 = ct->f140 * t524 * t599;
  expl_temp.f121 = ct->f327 * t524 * t599 * 8.0;
  expl_temp.f120 = ct->f425 * t518 * t603 * 10.0;
  expl_temp.f119 = ct->f142 * t518 * t603;
  expl_temp.f118 = ct->f148 * t523 * t597;
  expl_temp.f117 = ct->f407 * t518 * t603 * 8.0;
  expl_temp.f116 = ct->f150 * t517 * t601;
  expl_temp.f115 = ct->f322 * t523 * t597 * 10.0;
  expl_temp.f114 = ct->f139 * t523 * t597;
  expl_temp.f113 = ct->f305 * t523 * t597 * 8.0;
  expl_temp.f112 = ct->f397 * t517 * t601 * 10.0;
  expl_temp.f111 = ct->f141 * t517 * t601;
  expl_temp.f110 = ct->f147 * t522 * t595;
  expl_temp.f109 = ct->f364 * t517 * t601 * 8.0;
  expl_temp.f108 = ct->f149 * t516 * t599;
  expl_temp.f107 = ct->f297 * t522 * t595 * 10.0;
  expl_temp.f106 = ct->f138 * t522 * t595;
  expl_temp.f105 = ct->f287 * t522 * t595 * 8.0;
  expl_temp.f104 = ct->f353 * t516 * t599 * 10.0;
  expl_temp.f103 = ct->f140 * t516 * t599;
  expl_temp.f102 = ct->f327 * t516 * t599 * 8.0;
  expl_temp.f101 = ct->f148 * t515 * t597;
  expl_temp.f100 = ct->f322 * t515 * t597 * 10.0;
  expl_temp.f99 = ct->f139 * t515 * t597;
  expl_temp.f98 = ct->f305 * t515 * t597 * 8.0;
  expl_temp.f97 = ct->f147 * t514 * t595;
  expl_temp.f96 = ct->f66;
  expl_temp.f95 = ct->f297 * t514 * t595 * 10.0;
  expl_temp.f94 = ct->f138 * t514 * t595;
  expl_temp.f93 = ct->f287 * t514 * t595 * 8.0;
  expl_temp.f92 = t567 * t611;
  expl_temp.f91 = t566 * t611;
  expl_temp.f90 = t565 * t609;
  expl_temp.f89 = t564 * t607;
  expl_temp.f88 = t563 * t605;
  expl_temp.f87 = t557 * t609;
  expl_temp.f86 = t562 * t603;
  expl_temp.f85 = t556 * t607;
  expl_temp.f84 = t561 * t601;
  expl_temp.f83 = t555 * t605;
  expl_temp.f82 = t560 * t599;
  expl_temp.f81 = t554 * t603;
  expl_temp.f80 = t559 * t597;
  expl_temp.f79 = ct->f448 * t601;
  expl_temp.f78 = t558 * t595;
  expl_temp.f77 = ct->f447 * t599;
  expl_temp.f76 = ct->f446 * t597;
  expl_temp.f75 = ct->f63;
  expl_temp.f74 = ct->f445 * t595;
  expl_temp.f73 = ct->f62;
  expl_temp.f72 = expl_temp_tmp * -6.0;
  expl_temp.f71 = b_expl_temp_tmp * -6.0;
  expl_temp.f70 = c_expl_temp_tmp * -6.0;
  expl_temp.f69 = e_expl_temp_tmp * -6.0;
  expl_temp.f68 = g_expl_temp_tmp * -6.0;
  expl_temp.f67 = i_expl_temp_tmp * -6.0;
  expl_temp.f66 = k_expl_temp_tmp * -6.0;
  expl_temp.f65 = m_expl_temp_tmp * -6.0;
  expl_temp.f64 = o_expl_temp_tmp * -6.0;
  expl_temp.f63 = q_expl_temp_tmp * -6.0;
  expl_temp.f62 = s_expl_temp_tmp * -6.0;
  expl_temp.f61 = u_expl_temp_tmp * -6.0;
  expl_temp.f60 = w_expl_temp_tmp * -6.0;
  expl_temp.f59 = y_expl_temp_tmp * -6.0;
  expl_temp.f58 = bb_expl_temp_tmp * -6.0;
  expl_temp.f57 = db_expl_temp_tmp * -6.0;
  expl_temp.f56 = fb_expl_temp_tmp * -6.0;
  expl_temp.f55 = hb_expl_temp_tmp * -6.0;
  expl_temp.f54 = ct->f54;
  expl_temp.f53 = ct->f53;
  expl_temp.f52 = ct->f52;
  expl_temp.f51 = ct->f51;
  expl_temp.f50 = ct->f50;
  expl_temp.f49 = ct->f49;
  expl_temp.f48 = ct->f48;
  expl_temp.f47 = ct->f47;
  expl_temp.f46 = ct->f46;
  expl_temp.f45 = ct->f45;
  expl_temp.f44 = ct->f44;
  expl_temp.f43 = ct->f43;
  expl_temp.f42 = ct->f42;
  expl_temp.f41 = ct->f41;
  expl_temp.f40 = ct->f40;
  expl_temp.f39 = ct->f39;
  expl_temp.f38 = ct->f38;
  expl_temp.f37 = ct->f37;
  expl_temp.f36 = ct->f36;
  expl_temp.f35 = ct->f35;
  expl_temp.f34 = ct->f34;
  expl_temp.f33 = ct->f33;
  expl_temp.f32 = ct->f32;
  expl_temp.f31 = ct->f31;
  expl_temp.f30 = ct->f30;
  expl_temp.f29 = ct->f29;
  expl_temp.f28 = ct->f28;
  expl_temp.f27 = ct->f27;
  expl_temp.f26 = ct->f26;
  expl_temp.f25 = ct->f25;
  expl_temp.f24 = ct->f24;
  expl_temp.f23 = ct->f23;
  expl_temp.f22 = ct->f22;
  expl_temp.f21 = ct->f21;
  expl_temp.f20 = ct->f20;
  expl_temp.f19 = ct->f19;
  expl_temp.f18 = ct->f18;
  expl_temp.f17 = ct->f17;
  expl_temp.f16 = ct->f16;
  expl_temp.f15 = ct->f15;
  expl_temp.f14 = ct->f14;
  expl_temp.f13 = ct->f13;
  expl_temp.f12 = ct->f12;
  expl_temp.f11 = ct->f11;
  expl_temp.f10 = ct->f10;
  expl_temp.f9 = ct->f9;
  expl_temp.f8 = ct->f8;
  expl_temp.f7 = ct->f7;
  expl_temp.f6 = ct->f6;
  expl_temp.f5 = ct->f5;
  expl_temp.f4 = ct->f4;
  expl_temp.f3 = ct->f3;
  expl_temp.f2 = ct->f2;
  expl_temp.f1 = ct->f1;
  ft_2(&expl_temp, c_data, c_size, b_ceq_data, b_ceq_size, b_c_gradient_data,
       b_c_gradient_size, b_ceq_gradient_data, b_ceq_gradient_size);
  ceq_size[0] = 1;
  ceq_size[1] = b_ceq_size[1];
  memcpy(&ceq_data[0], &b_ceq_data[0], 18U * sizeof(double));
  c_gradient_size[0] = b_c_gradient_size[0];
  c_gradient_size[1] = b_c_gradient_size[1];
  memcpy(&c_gradient_data[0], &b_c_gradient_data[0], 2304U * sizeof(creal_T));
  ceq_gradient_size[0] = b_ceq_gradient_size[0];
  ceq_gradient_size[1] = b_ceq_gradient_size[1];
  memcpy(&ceq_gradient_data[0], &b_ceq_gradient_data[0], 324U * sizeof(double));
}

/*
 * Arguments    : const cell_15 *ct
 *                double c_data[]
 *                int c_size[2]
 *                double ceq_data[]
 *                int ceq_size[2]
 *                creal_T c_gradient_data[]
 *                int c_gradient_size[2]
 *                double ceq_gradient_data[]
 *                int ceq_gradient_size[2]
 * Return Type  : void
 */
static void ft_2(const cell_15 *ct, double c_data[], int c_size[2], double
                 ceq_data[], int ceq_size[2], creal_T c_gradient_data[], int
                 c_gradient_size[2], double ceq_gradient_data[], int
                 ceq_gradient_size[2])
{
  cell_16 expl_temp;
  creal_T c_gradient[2304];
  double b_c_tmp;
  double b_ct_im_tmp;
  double b_ct_re_tmp;
  double c_c_tmp;
  double c_ct_im_tmp;
  double c_ct_re_tmp;
  double c_tmp;
  double ct_im_tmp;
  double ct_re_tmp;
  double d_c_tmp;
  double d_ct_im_tmp;
  double d_ct_re_tmp;
  double e_ct_im_tmp;
  double e_ct_re_tmp;
  double f_ct_im_tmp;
  double f_ct_re_tmp;
  double g_ct_im_tmp;
  double g_ct_re_tmp;
  double h_ct_im_tmp;
  double h_ct_re_tmp;
  double i_ct_im_tmp;
  double i_ct_re_tmp;
  double j_ct_im_tmp;
  double j_ct_re_tmp;
  double k_ct_im_tmp;
  double k_ct_re_tmp;
  double l_ct_im_tmp;
  double l_ct_re_tmp;
  double m_ct_im_tmp;
  double m_ct_re_tmp;
  double n_ct_im_tmp;
  double n_ct_re_tmp;
  double o_ct_im_tmp;
  double o_ct_re_tmp;
  double p_ct_im_tmp;
  double p_ct_re_tmp;
  double q_ct_im_tmp;
  double q_ct_re_tmp;
  double r_ct_re_tmp;
  c_size[0] = 1;
  c_size[1] = 128;
  c_tmp = ct->f34 + ct->f231;
  c_data[0] = (((((((((c_tmp - ct->f27 * ct->f330) - ct->f29 * ct->f328) -
                     ct->f25 * ct->f334) - ct->f23 * ct->f337) + ct->f28 *
                   ct->f330) + ct->f30 * ct->f328) + ct->f26 * ct->f334) +
                ct->f24 * ct->f337) + ct->f32 * ct->f669) + ct->f230 * ct->f669;
  c_data[1] = (((((((((c_tmp - ct->f29 * ct->f333) - ct->f27 * ct->f335) -
                     ct->f25 * ct->f340) - ct->f23 * ct->f358) + ct->f30 *
                   ct->f333) + ct->f28 * ct->f335) + ct->f26 * ct->f340) +
                ct->f24 * ct->f358) + ct->f32 * ct->f670) + ct->f230 * ct->f670;
  c_data[2] = (((((((((c_tmp - ct->f29 * ct->f336) - ct->f27 * ct->f345) -
                     ct->f25 * ct->f361) - ct->f23 * ct->f364) + ct->f30 *
                   ct->f336) + ct->f28 * ct->f345) + ct->f26 * ct->f361) +
                ct->f24 * ct->f364) + ct->f32 * ct->f671) + ct->f230 * ct->f671;
  c_data[3] = (((((((((c_tmp - ct->f29 * ct->f354) - ct->f27 * ct->f362) -
                     ct->f25 * ct->f365) - ct->f23 * ct->f389) + ct->f30 *
                   ct->f354) + ct->f28 * ct->f362) + ct->f26 * ct->f365) +
                ct->f24 * ct->f389) + ct->f32 * ct->f672) + ct->f230 * ct->f672;
  c_data[4] = (((((((((c_tmp - ct->f29 * ct->f363) - ct->f27 * ct->f367) -
                     ct->f25 * ct->f400) - ct->f23 * ct->f433) + ct->f30 *
                   ct->f363) + ct->f28 * ct->f367) + ct->f26 * ct->f400) +
                ct->f24 * ct->f433) + ct->f32 * ct->f673) + ct->f230 * ct->f673;
  c_data[5] = (((((((((c_tmp - ct->f29 * ct->f378) - ct->f27 * ct->f411) -
                     ct->f25 * ct->f444) - ct->f23 * ct->f475) + ct->f30 *
                   ct->f378) + ct->f28 * ct->f411) + ct->f26 * ct->f444) +
                ct->f24 * ct->f475) + ct->f32 * ct->f674) + ct->f230 * ct->f674;
  c_data[6] = (((((((((c_tmp - ct->f29 * ct->f422) - ct->f27 * ct->f455) -
                     ct->f25 * ct->f476) - ct->f23 * ct->f484) + ct->f30 *
                   ct->f422) + ct->f28 * ct->f455) + ct->f26 * ct->f476) +
                ct->f24 * ct->f484) + ct->f32 * ct->f675) + ct->f230 * ct->f675;
  c_data[7] = (((((((((c_tmp - ct->f29 * ct->f466) - ct->f27 * ct->f477) -
                     ct->f23 * ct->f506) - ct->f25 * ct->f495) + ct->f30 *
                   ct->f466) + ct->f28 * ct->f477) + ct->f24 * ct->f506) +
                ct->f26 * ct->f495) + ct->f32 * ct->f676) + ct->f230 * ct->f676;
  c_data[8] = (ct->f226 + ct->f376) + ct->f379;
  c_data[9] = (ct->f226 + ct->f615) + ct->f618;
  c_data[10] = (ct->f226 + ct->f616) + ct->f620;
  c_data[11] = (ct->f226 + ct->f617) + ct->f622;
  c_data[12] = (ct->f226 + ct->f619) + ct->f624;
  c_data[13] = (ct->f226 + ct->f621) + ct->f626;
  c_data[14] = (ct->f226 + ct->f623) + ct->f628;
  c_data[15] = (ct->f226 + ct->f625) + ct->f629;
  c_data[16] = (ct->f226 + ct->f627) + ct->f630;
  c_data[17] = (ct->f226 + ct->f631) + ct->f632;
  c_data[18] = ct->f227;
  c_data[19] = ct->f227;
  c_data[20] = ct->f227;
  c_data[21] = ct->f227;
  c_data[22] = ct->f227;
  c_data[23] = ct->f227;
  c_data[24] = ct->f227;
  c_data[25] = ct->f227;
  c_data[26] = ct->f227;
  c_data[27] = ct->f227;
  c_data[28] = ct->f31 + ct->f228;
  c_data[29] = ((((ct->f31 + ct->f73) + ct->f228) + ct->f294) + ct->f295) +
    ct->f298;
  c_data[30] = ((((ct->f31 + ct->f96) + ct->f228) + ct->f296) + ct->f299) +
    ct->f301;
  c_data[31] = ((((ct->f31 + ct->f127) + ct->f228) + ct->f300) + ct->f302) +
    ct->f305;
  c_data[32] = ((((ct->f31 + ct->f161) + ct->f228) + ct->f304) + ct->f306) +
    ct->f308;
  c_data[33] = ((((ct->f31 + ct->f163) + ct->f228) + ct->f307) + ct->f309) +
    ct->f311;
  c_data[34] = ((((ct->f31 + ct->f165) + ct->f228) + ct->f310) + ct->f312) +
    ct->f315;
  c_data[35] = ((((ct->f31 + ct->f167) + ct->f228) + ct->f314) + ct->f316) +
    ct->f318;
  c_data[36] = ((((ct->f31 + ct->f179) + ct->f228) + ct->f317) + ct->f319) +
    ct->f320;
  c_data[37] = ((((ct->f31 + ct->f219) + ct->f228) + ct->f323) + ct->f324) +
    ct->f325;
  c_data[38] = (ct->f8 - ct->f376) - ct->f379;
  c_data[39] = (ct->f8 - ct->f615) - ct->f618;
  c_data[40] = (ct->f8 - ct->f616) - ct->f620;
  c_data[41] = (ct->f8 - ct->f617) - ct->f622;
  c_data[42] = (ct->f8 - ct->f619) - ct->f624;
  c_data[43] = (ct->f8 - ct->f621) - ct->f626;
  c_data[44] = (ct->f8 - ct->f623) - ct->f628;
  c_data[45] = (ct->f8 - ct->f625) - ct->f629;
  c_data[46] = (ct->f8 - ct->f627) - ct->f630;
  c_data[47] = (ct->f8 - ct->f631) - ct->f632;
  c_data[48] = ct->f10;
  c_data[49] = ct->f10;
  c_data[50] = ct->f10;
  c_data[51] = ct->f10;
  c_data[52] = ct->f10;
  c_data[53] = ct->f10;
  c_data[54] = ct->f10;
  c_data[55] = ct->f10;
  c_data[56] = ct->f10;
  c_data[57] = ct->f10;
  c_tmp = ct->f12 + ct->f230;
  c_data[58] = c_tmp;
  c_data[59] = (((c_tmp - ct->f25 * ct->f330 * 4.0) - ct->f27 * ct->f328 * 3.0)
                - ct->f23 * ct->f334 * 5.0) - ct->f29 * ct->f669 * 2.0;
  c_data[60] = (((c_tmp - ct->f27 * ct->f333 * 3.0) - ct->f25 * ct->f335 * 4.0)
                - ct->f23 * ct->f340 * 5.0) - ct->f29 * ct->f670 * 2.0;
  c_data[61] = (((c_tmp - ct->f27 * ct->f336 * 3.0) - ct->f25 * ct->f345 * 4.0)
                - ct->f23 * ct->f361 * 5.0) - ct->f29 * ct->f671 * 2.0;
  c_data[62] = (((c_tmp - ct->f27 * ct->f354 * 3.0) - ct->f25 * ct->f362 * 4.0)
                - ct->f23 * ct->f365 * 5.0) - ct->f29 * ct->f672 * 2.0;
  c_data[63] = (((c_tmp - ct->f27 * ct->f363 * 3.0) - ct->f25 * ct->f367 * 4.0)
                - ct->f23 * ct->f400 * 5.0) - ct->f29 * ct->f673 * 2.0;
  c_data[64] = (((c_tmp - ct->f27 * ct->f378 * 3.0) - ct->f25 * ct->f411 * 4.0)
                - ct->f23 * ct->f444 * 5.0) - ct->f29 * ct->f674 * 2.0;
  c_data[65] = (((c_tmp - ct->f27 * ct->f422 * 3.0) - ct->f25 * ct->f455 * 4.0)
                - ct->f23 * ct->f476 * 5.0) - ct->f29 * ct->f675 * 2.0;
  c_data[66] = (((c_tmp - ct->f27 * ct->f466 * 3.0) - ct->f25 * ct->f477 * 4.0)
                - ct->f23 * ct->f495 * 5.0) - ct->f29 * ct->f676 * 2.0;
  c_data[67] = (((c_tmp - ct->f23 * ct->f511 * 5.0) - ct->f25 * ct->f510 * 4.0)
                - ct->f27 * ct->f509 * 3.0) - ct->f29 * ct->f677 * 2.0;
  c_data[68] = (ct->f223 + ct->f381) + ct->f384;
  c_data[69] = (ct->f223 + ct->f416 * ct->f577) + ct->f425 * ct->f580;
  c_data[70] = (ct->f223 + ct->f417 * ct->f578) + ct->f426 * ct->f582;
  c_data[71] = (ct->f223 + ct->f418 * ct->f579) + ct->f427 * ct->f585;
  c_data[72] = (ct->f223 + ct->f419 * ct->f581) + ct->f428 * ct->f587;
  c_data[73] = (ct->f223 + ct->f420 * ct->f583) + ct->f429 * ct->f589;
  c_data[74] = (ct->f223 + ct->f421 * ct->f586) + ct->f430 * ct->f591;
  c_data[75] = (ct->f223 + ct->f423 * ct->f588) + ct->f431 * ct->f592;
  c_data[76] = (ct->f223 + ct->f424 * ct->f590) + ct->f432 * ct->f593;
  c_data[77] = (ct->f223 + ct->f434 * ct->f594) + ct->f435 * ct->f595;
  c_data[78] = (ct->f224 + ct->f383) + ct->f19 * ct->f372;
  c_data[79] = (ct->f224 + ct->f416 * ct->f580) + ct->f425 * -ct->f577;
  c_data[80] = (ct->f224 + ct->f417 * ct->f582) + ct->f426 * -ct->f578;
  c_data[81] = (ct->f224 + ct->f418 * ct->f585) + ct->f427 * -ct->f579;
  c_data[82] = (ct->f224 + ct->f419 * ct->f587) + ct->f428 * -ct->f581;
  c_data[83] = (ct->f224 + ct->f420 * ct->f589) + ct->f429 * -ct->f583;
  c_data[84] = (ct->f224 + ct->f421 * ct->f591) + ct->f430 * -ct->f586;
  c_data[85] = (ct->f224 + ct->f423 * ct->f592) + ct->f431 * -ct->f588;
  c_data[86] = (ct->f224 + ct->f424 * ct->f593) + ct->f432 * -ct->f590;
  c_data[87] = (ct->f224 + ct->f434 * ct->f595) + ct->f435 * -ct->f594;
  c_data[88] = ct->f584 + ct->f225;
  c_data[89] = (((ct->f584 + ct->f158) + ct->f225) + ct->f338) + ct->f348;
  c_data[90] = (((ct->f584 + ct->f162) + ct->f225) + ct->f339) + ct->f349;
  c_data[91] = (((ct->f584 + ct->f164) + ct->f225) + ct->f341) + ct->f350;
  c_data[92] = (((ct->f584 + ct->f166) + ct->f225) + ct->f342) + ct->f351;
  c_data[93] = (((ct->f584 + ct->f168) + ct->f225) + ct->f343) + ct->f352;
  c_data[94] = (((ct->f584 + ct->f190) + ct->f225) + ct->f344) + ct->f355;
  c_data[95] = (((ct->f584 + ct->f202) + ct->f225) + ct->f346) + ct->f356;
  c_data[96] = (((ct->f584 + ct->f213) + ct->f225) + ct->f347) + ct->f357;
  c_data[97] = (((ct->f584 + ct->f220) + ct->f225) + ct->f359) + ct->f360;
  c_data[98] = (ct->f2 + ct->f43 * ct->f372) + ct->f19 * ct->f374;
  c_data[99] = (ct->f2 + ct->f416 * -ct->f577) + ct->f425 * -ct->f580;
  c_data[100] = (ct->f2 + ct->f417 * -ct->f578) + ct->f426 * -ct->f582;
  c_data[101] = (ct->f2 + ct->f418 * -ct->f579) + ct->f427 * -ct->f585;
  c_data[102] = (ct->f2 + ct->f419 * -ct->f581) + ct->f428 * -ct->f587;
  c_data[103] = (ct->f2 + ct->f420 * -ct->f583) + ct->f429 * -ct->f589;
  c_data[104] = (ct->f2 + ct->f421 * -ct->f586) + ct->f430 * -ct->f591;
  c_data[105] = (ct->f2 + ct->f423 * -ct->f588) + ct->f431 * -ct->f592;
  c_data[106] = (ct->f2 + ct->f424 * -ct->f590) + ct->f432 * -ct->f593;
  c_data[107] = (ct->f2 + ct->f434 * -ct->f594) + ct->f435 * -ct->f595;
  c_data[108] = (ct->f4 + ct->f382) + ct->f17 * ct->f373;
  c_data[109] = (ct->f4 + ct->f425 * ct->f577) + ct->f416 * -ct->f580;
  c_data[110] = (ct->f4 + ct->f426 * ct->f578) + ct->f417 * -ct->f582;
  c_data[111] = (ct->f4 + ct->f427 * ct->f579) + ct->f418 * -ct->f585;
  c_data[112] = (ct->f4 + ct->f428 * ct->f581) + ct->f419 * -ct->f587;
  c_data[113] = (ct->f4 + ct->f429 * ct->f583) + ct->f420 * -ct->f589;
  c_data[114] = (ct->f4 + ct->f430 * ct->f586) + ct->f421 * -ct->f591;
  c_data[115] = (ct->f4 + ct->f431 * ct->f588) + ct->f423 * -ct->f592;
  c_data[116] = (ct->f4 + ct->f432 * ct->f590) + ct->f424 * -ct->f593;
  c_data[117] = (ct->f4 + ct->f435 * ct->f594) + ct->f434 * -ct->f595;
  c_tmp = ct->f6 + ct->f229;
  c_data[118] = c_tmp;
  c_data[119] = ((c_tmp - ct->f23 * ct->f330 * 20.0) - ct->f25 * ct->f328 * 12.0)
    - ct->f27 * ct->f669 * 6.0;
  c_data[120] = ((c_tmp - ct->f25 * ct->f333 * 12.0) - ct->f23 * ct->f335 * 20.0)
    - ct->f27 * ct->f670 * 6.0;
  c_data[121] = ((c_tmp - ct->f25 * ct->f336 * 12.0) - ct->f23 * ct->f345 * 20.0)
    - ct->f27 * ct->f671 * 6.0;
  c_data[122] = ((c_tmp - ct->f25 * ct->f354 * 12.0) - ct->f23 * ct->f362 * 20.0)
    - ct->f27 * ct->f672 * 6.0;
  c_data[123] = ((c_tmp - ct->f25 * ct->f363 * 12.0) - ct->f23 * ct->f367 * 20.0)
    - ct->f27 * ct->f673 * 6.0;
  c_data[124] = ((c_tmp - ct->f25 * ct->f378 * 12.0) - ct->f23 * ct->f411 * 20.0)
    - ct->f27 * ct->f674 * 6.0;
  c_data[125] = ((c_tmp - ct->f25 * ct->f422 * 12.0) - ct->f23 * ct->f455 * 20.0)
    - ct->f27 * ct->f675 * 6.0;
  c_data[126] = ((c_tmp - ct->f25 * ct->f466 * 12.0) - ct->f23 * ct->f477 * 20.0)
    - ct->f27 * ct->f676 * 6.0;
  b_c_tmp = ct->f23 * ct->f510 * 20.0;
  c_c_tmp = ct->f25 * ct->f509 * 12.0;
  d_c_tmp = ct->f27 * ct->f677 * 6.0;
  c_data[127] = ((c_tmp - b_c_tmp) - c_c_tmp) - d_c_tmp;
  ceq_size[0] = 1;
  ceq_size[1] = 18;
  ceq_data[0] = -ct->f53 * (ct->f13 - ct->f45);
  ceq_data[1] = -ct->f53 * (ct->f14 - ct->f21);
  ceq_data[2] = -ct->f53 * (ct->f15 + ct->f231);
  ceq_data[3] = -ct->f54 * (ct->f7 - ct->f43);
  ceq_data[4] = -ct->f54 * (ct->f9 - ct->f19);
  ceq_data[5] = -ct->f54 * (ct->f11 + ct->f230);
  ceq_data[6] = -ct->f16 * (ct->f1 - ct->f353);
  ceq_data[7] = -ct->f16 * (ct->f3 - ct->f483);
  ceq_data[8] = -ct->f16 * (ct->f5 + ct->f229);
  ceq_data[9] = ct->f53 * (((((((((((ct->f45 - ct->f46) + ct->f35 * ct->f512) +
    ct->f37 * ct->f511) + ct->f39 * ct->f510) + ct->f41 * ct->f509) - ct->f36 *
    ct->f512) - ct->f38 * ct->f511) - ct->f40 * ct->f510) - ct->f42 * ct->f509)
    + ct->f43 * ct->f677) - ct->f44 * ct->f677);
  ceq_data[10] = ct->f53 * (((((((((((ct->f21 - ct->f22) + ct->f47 * ct->f512) +
    ct->f49 * ct->f511) + ct->f51 * ct->f510) + ct->f17 * ct->f509) - ct->f48 *
    ct->f512) - ct->f50 * ct->f511) - ct->f52 * ct->f510) - ct->f18 * ct->f509)
    + ct->f19 * ct->f677) - ct->f20 * ct->f677);
  ceq_data[11] = ct->f53 * (((((((((((ct->f33 - ct->f34) + ct->f23 * ct->f512) +
    ct->f25 * ct->f511) + ct->f27 * ct->f510) + ct->f29 * ct->f509) - ct->f24 *
    ct->f512) - ct->f26 * ct->f511) - ct->f28 * ct->f510) - ct->f30 * ct->f509)
    + ct->f31 * ct->f677) - ct->f32 * ct->f677);
  ceq_data[12] = -ct->f54 * (((((ct->f44 - ct->f434) + ct->f36 * ct->f511 * 5.0)
    + ct->f38 * ct->f510 * 4.0) + ct->f40 * ct->f509 * 3.0) + ct->f42 * ct->f677
    * 2.0);
  ceq_data[13] = -ct->f54 * (((((ct->f20 - ct->f435) + ct->f48 * ct->f511 * 5.0)
    + ct->f50 * ct->f510 * 4.0) + ct->f52 * ct->f509 * 3.0) + ct->f18 * ct->f677
    * 2.0);
  ceq_data[14] = ct->f54 * (((((((((ct->f31 - ct->f32) + ct->f219) + ct->f323) +
    ct->f324) + ct->f325) - ct->f24 * ct->f511 * 5.0) - ct->f26 * ct->f510 * 4.0)
    - ct->f28 * ct->f509 * 3.0) - ct->f30 * ct->f677 * 2.0);
  ceq_data[15] = -ct->f16 * (((((((ct->f42 * 2.0 - ct->f353) - ct->f35 *
    ct->f510 * 20.0) - ct->f37 * ct->f509 * 12.0) + ct->f40 * ct->f322) +
    ct->f36 * ct->f285) + ct->f38 * ct->f284) - ct->f39 * ct->f677 * 6.0);
  ceq_data[16] = -ct->f16 * (((((((ct->f18 * 2.0 - ct->f483) - ct->f47 *
    ct->f510 * 20.0) - ct->f49 * ct->f509 * 12.0) + ct->f52 * ct->f322) +
    ct->f48 * ct->f285) + ct->f50 * ct->f284) - ct->f51 * ct->f677 * 6.0);
  ceq_data[17] = -ct->f16 * (((((((ct->f30 * 2.0 + ct->f229) - b_c_tmp) -
    c_c_tmp) + ct->f28 * ct->f322) + ct->f24 * ct->f285) + ct->f26 * ct->f284) -
    d_c_tmp);
  expl_temp.f955 = ct->f676;
  expl_temp.f954 = ct->f675;
  expl_temp.f953 = ct->f674;
  expl_temp.f952 = ct->f673;
  expl_temp.f951 = ct->f672;
  expl_temp.f950 = ct->f671;
  expl_temp.f949 = ct->f670;
  expl_temp.f948 = ct->f669;
  expl_temp.f947 = ct->f257 * ct->f592;
  expl_temp.f946 = ct->f253 * ct->f592;
  expl_temp.f945 = ct->f248 * ct->f592;
  expl_temp.f944 = ct->f258 * ct->f590;
  expl_temp.f943 = ct->f255 * ct->f591;
  expl_temp.f942 = ct->f256 * ct->f590;
  expl_temp.f941 = ct->f250 * ct->f591;
  expl_temp.f940 = ct->f251 * ct->f590;
  expl_temp.f939 = ct->f245 * ct->f591;
  expl_temp.f938 = ct->f257 * ct->f588;
  expl_temp.f937 = ct->f252 * ct->f589;
  expl_temp.f936 = ct->f253 * ct->f588;
  expl_temp.f935 = ct->f247 * ct->f589;
  expl_temp.f934 = ct->f248 * ct->f588;
  expl_temp.f933 = ct->f241 * ct->f589;
  expl_temp.f932 = ct->f255 * ct->f586;
  expl_temp.f931 = ct->f249 * ct->f587;
  expl_temp.f930 = ct->f250 * ct->f586;
  expl_temp.f929 = ct->f244 * ct->f587;
  expl_temp.f928 = ct->f245 * ct->f586;
  expl_temp.f927 = ct->f238 * ct->f587;
  expl_temp.f926 = ct->f252 * ct->f583;
  expl_temp.f925 = ct->f246 * ct->f585;
  expl_temp.f924 = ct->f247 * ct->f583;
  expl_temp.f923 = ct->f240 * ct->f585;
  expl_temp.f922 = ct->f241 * ct->f583;
  expl_temp.f921 = ct->f236 * ct->f585;
  expl_temp.f920 = ct->f249 * ct->f581;
  expl_temp.f919 = ct->f242 * ct->f582;
  expl_temp.f918 = ct->f244 * ct->f581;
  expl_temp.f917 = ct->f237 * ct->f582;
  expl_temp.f916 = ct->f238 * ct->f581;
  expl_temp.f915 = ct->f234 * ct->f582;
  expl_temp.f914 = ct->f246 * ct->f579;
  expl_temp.f913 = ct->f239 * ct->f580;
  expl_temp.f912 = ct->f240 * ct->f579;
  expl_temp.f911 = ct->f235 * ct->f580;
  expl_temp.f910 = ct->f236 * ct->f579;
  expl_temp.f909 = ct->f233 * ct->f580;
  expl_temp.f908 = ct->f242 * ct->f578;
  expl_temp.f907 = ct->f237 * ct->f578;
  expl_temp.f906 = ct->f234 * ct->f578;
  expl_temp.f905 = ct->f239 * ct->f577;
  expl_temp.f904 = ct->f235 * ct->f577;
  expl_temp.f903 = ct->f233 * ct->f577;
  expl_temp.f902 = ct->f668;
  expl_temp.f901 = ct->f667;
  expl_temp.f900 = ct->f666;
  expl_temp.f899 = ct->f665;
  expl_temp.f898 = ct->f664;
  expl_temp.f897 = ct->f663;
  expl_temp.f896 = ct->f662;
  expl_temp.f895 = ct->f661;
  expl_temp.f894 = ct->f660;
  expl_temp.f893 = ct->f659;
  expl_temp.f892 = ct->f658;
  expl_temp.f891 = ct->f657;
  expl_temp.f890 = ct->f656;
  expl_temp.f889 = ct->f655;
  expl_temp.f888 = ct->f654;
  expl_temp.f887 = ct->f653;
  expl_temp.f886 = ct->f652;
  expl_temp.f885 = ct->f651;
  expl_temp.f884 = ct->f650;
  expl_temp.f883 = ct->f649;
  expl_temp.f882 = ct->f648;
  expl_temp.f881 = ct->f647;
  expl_temp.f880 = ct->f646;
  expl_temp.f879 = ct->f645;
  expl_temp.f878 = ct->f644;
  expl_temp.f877 = ct->f643;
  expl_temp.f876 = ct->f642;
  expl_temp.f875 = ct->f641;
  expl_temp.f874 = ct->f640;
  expl_temp.f873 = ct->f639;
  expl_temp.f872 = ct->f638;
  expl_temp.f871 = ct->f637;
  expl_temp.f870 = ct->f636;
  expl_temp.f869 = ct->f635;
  expl_temp.f868 = ct->f634;
  expl_temp.f867 = ct->f633;
  expl_temp.f866 = -ct->f613;
  expl_temp.f865 = ct->f614;
  expl_temp.f864 = -ct->f612;
  expl_temp.f863 = -ct->f611;
  expl_temp.f862 = -ct->f610;
  expl_temp.f861 = -ct->f609;
  expl_temp.f860 = -ct->f608;
  expl_temp.f859 = -ct->f607;
  expl_temp.f858 = -ct->f606;
  expl_temp.f857 = -ct->f605;
  expl_temp.f856 = -ct->f604;
  expl_temp.f855 = -ct->f603;
  expl_temp.f854 = -ct->f602;
  expl_temp.f853 = -ct->f601;
  expl_temp.f852 = -ct->f600;
  expl_temp.f851 = -ct->f599;
  expl_temp.f850 = -ct->f598;
  expl_temp.f849 = -ct->f597;
  expl_temp.f848 = -ct->f596;
  expl_temp.f847 = ct->f595 * ct->f677 * -2.0;
  expl_temp.f846 = ct->f594 * ct->f677 * -2.0;
  expl_temp.f845 = ct->f593 * ct->f676 * -2.0;
  expl_temp.f844 = ct->f592 * ct->f675 * -2.0;
  expl_temp.f843 = ct->f590 * ct->f676 * -2.0;
  expl_temp.f842 = ct->f591 * ct->f674 * -2.0;
  expl_temp.f841 = ct->f588 * ct->f675 * -2.0;
  expl_temp.f840 = ct->f589 * ct->f673 * -2.0;
  expl_temp.f839 = ct->f586 * ct->f674 * -2.0;
  expl_temp.f838 = ct->f587 * ct->f672 * -2.0;
  expl_temp.f837 = ct->f583 * ct->f673 * -2.0;
  expl_temp.f836 = ct->f585 * ct->f671 * -2.0;
  expl_temp.f835 = ct->f581 * ct->f672 * -2.0;
  expl_temp.f834 = ct->f582 * ct->f670 * -2.0;
  expl_temp.f833 = ct->f579 * ct->f671 * -2.0;
  expl_temp.f832 = ct->f580 * ct->f669 * -2.0;
  expl_temp.f831 = ct->f578 * ct->f670 * -2.0;
  expl_temp.f830 = ct->f577 * ct->f669 * -2.0;
  expl_temp.f829 = ct->f613;
  expl_temp.f828 = ct->f612;
  expl_temp.f827 = ct->f611;
  expl_temp.f826 = ct->f610;
  expl_temp.f825 = ct->f609;
  expl_temp.f824 = ct->f608;
  expl_temp.f823 = ct->f607;
  expl_temp.f822 = ct->f606;
  expl_temp.f821 = ct->f605;
  expl_temp.f820 = ct->f604;
  expl_temp.f819 = ct->f603;
  expl_temp.f818 = ct->f602;
  expl_temp.f817 = ct->f601;
  expl_temp.f816 = ct->f600;
  expl_temp.f815 = ct->f599;
  expl_temp.f814 = ct->f598;
  expl_temp.f813 = ct->f597;
  expl_temp.f812 = ct->f596;
  expl_temp.f811 = ct->f313 * ct->f595;
  expl_temp.f810 = ct->f313 * ct->f594;
  expl_temp.f809 = ct->f291 * ct->f593;
  expl_temp.f808 = ct->f289 * ct->f592;
  expl_temp.f807 = ct->f291 * ct->f590;
  expl_temp.f806 = ct->f287 * ct->f591;
  expl_temp.f805 = ct->f289 * ct->f588;
  expl_temp.f804 = ct->f265 * ct->f589;
  expl_temp.f803 = ct->f287 * ct->f586;
  expl_temp.f802 = ct->f243 * ct->f587;
  expl_temp.f801 = ct->f265 * ct->f583;
  expl_temp.f800 = ct->f232 * ct->f585;
  expl_temp.f799 = ct->f243 * ct->f581;
  expl_temp.f798 = ct->f221 * ct->f582;
  expl_temp.f797 = ct->f232 * ct->f579;
  expl_temp.f796 = ct->f201 * ct->f580;
  expl_temp.f795 = ct->f221 * ct->f578;
  expl_temp.f794 = ct->f201 * ct->f577;
  expl_temp.f793 = -ct->f595;
  expl_temp.f792 = -ct->f594;
  expl_temp.f791 = -ct->f593;
  expl_temp.f790 = -ct->f592;
  expl_temp.f789 = -ct->f591;
  expl_temp.f788 = -ct->f590;
  expl_temp.f787 = -ct->f589;
  expl_temp.f786 = -ct->f588;
  expl_temp.f785 = -ct->f587;
  expl_temp.f784 = -ct->f586;
  expl_temp.f783 = -ct->f585;
  expl_temp.f782 = -ct->f583;
  expl_temp.f781 = -ct->f582;
  expl_temp.f780 = -ct->f581;
  expl_temp.f779 = -ct->f580;
  expl_temp.f778 = -ct->f579;
  expl_temp.f777 = -ct->f578;
  expl_temp.f776 = -ct->f577;
  expl_temp.f775 = ct->f595;
  expl_temp.f774 = ct->f594;
  expl_temp.f773 = ct->f593;
  expl_temp.f772 = ct->f592;
  expl_temp.f771 = ct->f591;
  expl_temp.f770 = ct->f590;
  expl_temp.f769 = ct->f589;
  expl_temp.f768 = ct->f588;
  expl_temp.f767 = ct->f587;
  expl_temp.f766 = ct->f586;
  expl_temp.f765 = ct->f585;
  expl_temp.f764 = ct->f583;
  expl_temp.f763 = ct->f582;
  expl_temp.f762 = ct->f581;
  expl_temp.f761 = ct->f580;
  expl_temp.f760 = ct->f579;
  expl_temp.f759 = ct->f578;
  expl_temp.f758 = ct->f577;
  expl_temp.f757 = ct->f576;
  expl_temp.f756 = ct->f575;
  expl_temp.f755 = ct->f574;
  expl_temp.f754 = ct->f573;
  expl_temp.f753 = ct->f572;
  expl_temp.f752 = ct->f571;
  expl_temp.f751 = ct->f570;
  expl_temp.f750 = ct->f569;
  expl_temp.f749 = ct->f568;
  expl_temp.f748 = ct->f567;
  expl_temp.f747 = ct->f566;
  expl_temp.f746 = ct->f565;
  expl_temp.f745 = ct->f564;
  expl_temp.f744 = ct->f563;
  expl_temp.f743 = ct->f562;
  expl_temp.f742 = ct->f561;
  expl_temp.f741 = ct->f560;
  expl_temp.f740 = ct->f559;
  expl_temp.f739 = ct->f558;
  expl_temp.f738 = ct->f557;
  expl_temp.f737 = ct->f556;
  expl_temp.f736 = ct->f555;
  expl_temp.f735 = ct->f554;
  expl_temp.f734 = ct->f553;
  expl_temp.f733 = ct->f552;
  expl_temp.f732 = ct->f551;
  expl_temp.f731 = ct->f550;
  expl_temp.f730 = ct->f549;
  expl_temp.f729 = ct->f548;
  expl_temp.f728 = ct->f547;
  expl_temp.f727 = ct->f546;
  expl_temp.f726 = ct->f545;
  expl_temp.f725 = ct->f544;
  expl_temp.f724 = ct->f543;
  expl_temp.f723 = ct->f542;
  expl_temp.f722 = ct->f541;
  expl_temp.f721 = ct->f540;
  expl_temp.f720 = ct->f539;
  expl_temp.f719 = ct->f538;
  expl_temp.f718 = ct->f537;
  expl_temp.f717 = ct->f536;
  expl_temp.f716 = ct->f535;
  expl_temp.f715 = ct->f534;
  expl_temp.f714 = ct->f533;
  expl_temp.f713 = ct->f532;
  expl_temp.f712 = ct->f531;
  expl_temp.f711 = ct->f530;
  expl_temp.f710 = ct->f529;
  expl_temp.f709 = ct->f528;
  expl_temp.f708 = ct->f527;
  expl_temp.f707 = ct->f526;
  expl_temp.f706 = ct->f525;
  expl_temp.f705 = ct->f524;
  expl_temp.f704 = ct->f523;
  expl_temp.f703 = ct->f522;
  expl_temp.f702 = ct->f521;
  expl_temp.f701 = ct->f520;
  expl_temp.f700 = ct->f519;
  expl_temp.f699 = ct->f518;
  expl_temp.f698 = ct->f517;
  expl_temp.f697 = ct->f516;
  expl_temp.f696 = ct->f515;
  expl_temp.f695 = ct->f514;
  expl_temp.f694 = ct->f513;
  expl_temp.f693 = ct->f509;
  expl_temp.f692 = ct->f508;
  expl_temp.f691 = ct->f507;
  expl_temp.f690 = ct->f506;
  expl_temp.f689 = ct->f505;
  expl_temp.f688 = ct->f504;
  expl_temp.f687 = ct->f503;
  expl_temp.f686 = ct->f502;
  expl_temp.f685 = ct->f501;
  expl_temp.f684 = ct->f500;
  expl_temp.f683 = ct->f499;
  expl_temp.f682 = ct->f498;
  expl_temp.f681 = ct->f497;
  expl_temp.f680 = ct->f496;
  expl_temp.f679 = ct->f495;
  expl_temp.f678 = ct->f494;
  expl_temp.f677 = ct->f493;
  expl_temp.f676 = ct->f492;
  expl_temp.f675 = ct->f491;
  expl_temp.f674 = ct->f490;
  expl_temp.f673 = ct->f489;
  expl_temp.f672 = ct->f488;
  expl_temp.f671 = ct->f487;
  expl_temp.f670 = ct->f486;
  expl_temp.f669 = ct->f485;
  expl_temp.f668 = ct->f484;
  expl_temp.f667 = ct->f482;
  expl_temp.f666 = ct->f481;
  expl_temp.f665 = ct->f480;
  expl_temp.f664 = ct->f479;
  expl_temp.f663 = ct->f478;
  expl_temp.f662 = ct->f477;
  expl_temp.f661 = ct->f476;
  expl_temp.f660 = ct->f475;
  expl_temp.f659 = ct->f466;
  expl_temp.f658 = ct->f455;
  expl_temp.f657 = ct->f454;
  expl_temp.f656 = ct->f453;
  expl_temp.f655 = ct->f452;
  expl_temp.f654 = ct->f451;
  expl_temp.f653 = ct->f450;
  expl_temp.f652 = ct->f449;
  expl_temp.f651 = ct->f448;
  expl_temp.f650 = ct->f447;
  expl_temp.f649 = ct->f446;
  expl_temp.f648 = ct->f445;
  expl_temp.f647 = ct->f444;
  expl_temp.f646 = ct->f443;
  expl_temp.f645 = ct->f442;
  expl_temp.f644 = ct->f441;
  expl_temp.f643 = ct->f440;
  expl_temp.f642 = ct->f439;
  expl_temp.f641 = ct->f438;
  expl_temp.f640 = ct->f437;
  expl_temp.f639 = ct->f436;
  expl_temp.f638 = ct->f435;
  expl_temp.f637 = ct->f434;
  expl_temp.f636 = ct->f433;
  expl_temp.f635 = ct->f432;
  expl_temp.f634 = ct->f431;
  expl_temp.f633 = ct->f430;
  expl_temp.f632 = ct->f429;
  expl_temp.f631 = ct->f428;
  expl_temp.f630 = ct->f427;
  expl_temp.f629 = ct->f426;
  expl_temp.f628 = ct->f425;
  expl_temp.f627 = ct->f424;
  expl_temp.f626 = ct->f423;
  expl_temp.f625 = ct->f422;
  expl_temp.f624 = ct->f421;
  expl_temp.f623 = ct->f420;
  expl_temp.f622 = ct->f419;
  expl_temp.f621 = ct->f418;
  expl_temp.f620 = ct->f417;
  expl_temp.f619 = ct->f416;
  expl_temp.f618 = ct->f415;
  expl_temp.f617 = ct->f414;
  expl_temp.f616 = ct->f413;
  expl_temp.f615 = ct->f412;
  expl_temp.f614 = ct->f411;
  expl_temp.f613 = ct->f410;
  expl_temp.f612 = ct->f409;
  expl_temp.f611 = ct->f408;
  expl_temp.f610 = ct->f407;
  expl_temp.f609 = ct->f406;
  expl_temp.f608 = ct->f405;
  expl_temp.f607 = ct->f404;
  expl_temp.f606 = ct->f403;
  expl_temp.f605 = ct->f402;
  expl_temp.f604 = ct->f401;
  expl_temp.f603 = ct->f400;
  expl_temp.f602 = ct->f399;
  expl_temp.f601 = ct->f398;
  expl_temp.f600 = ct->f397;
  expl_temp.f599 = ct->f396;
  expl_temp.f598 = ct->f395;
  expl_temp.f597 = ct->f394;
  expl_temp.f596 = ct->f393;
  expl_temp.f595 = ct->f392;
  expl_temp.f594 = ct->f391;
  expl_temp.f593 = ct->f390;
  expl_temp.f592 = ct->f389;
  expl_temp.f591 = ct->f388;
  expl_temp.f590 = ct->f387;
  expl_temp.f589 = ct->f386;
  expl_temp.f588 = ct->f385;
  expl_temp.f587 = ct->f380;
  expl_temp.f586 = ct->f378;
  expl_temp.f585 = ct->f377;
  expl_temp.f584 = ct->f375;
  expl_temp.f583 = ct->f374;
  expl_temp.f582 = ct->f373;
  expl_temp.f581 = ct->f372;
  expl_temp.f580 = ct->f371;
  expl_temp.f579 = ct->f370;
  expl_temp.f578 = ct->f369;
  expl_temp.f577 = ct->f368;
  expl_temp.f576 = ct->f367;
  expl_temp.f575 = ct->f366;
  expl_temp.f574 = ct->f365;
  expl_temp.f573 = ct->f364;
  expl_temp.f572 = ct->f363;
  expl_temp.f571 = ct->f362;
  expl_temp.f570 = ct->f361;
  expl_temp.f569 = ct->f358;
  expl_temp.f568 = ct->f354;
  expl_temp.f567 = ct->f345;
  expl_temp.f566 = ct->f340;
  expl_temp.f565 = ct->f337;
  expl_temp.f564 = ct->f336;
  expl_temp.f563 = ct->f335;
  expl_temp.f562 = ct->f334;
  expl_temp.f561 = ct->f333;
  expl_temp.f560 = ct->f332;
  expl_temp.f559 = ct->f331;
  expl_temp.f558 = ct->f330;
  expl_temp.f557 = ct->f329;
  expl_temp.f556 = ct->f328;
  expl_temp.f555 = ct->f327;
  expl_temp.f554 = ct->f326;
  expl_temp.f553 = ct->f322;
  expl_temp.f552 = ct->f321;
  expl_temp.f551 = ct->f313;
  expl_temp.f550 = ct->f303;
  expl_temp.f549 = ct->f297;
  expl_temp.f548 = ct->f293;
  expl_temp.f547 = ct->f292;
  expl_temp.f546 = ct->f291;
  expl_temp.f545 = ct->f290;
  expl_temp.f544 = ct->f289;
  expl_temp.f543 = ct->f288;
  expl_temp.f542 = ct->f287;
  expl_temp.f541 = ct->f286;
  expl_temp.f540 = ct->f285;
  expl_temp.f539 = ct->f284;
  expl_temp.f538 = ct->f283;
  expl_temp.f537 = ct->f282;
  expl_temp.f536 = ct->f281;
  expl_temp.f535 = ct->f280;
  expl_temp.f534 = ct->f279;
  expl_temp.f533 = ct->f278;
  expl_temp.f532 = ct->f277;
  expl_temp.f531 = ct->f276;
  expl_temp.f530 = ct->f275;
  expl_temp.f529 = ct->f274;
  expl_temp.f528 = ct->f273;
  expl_temp.f527 = ct->f272;
  expl_temp.f526 = ct->f271;
  expl_temp.f525 = ct->f270;
  expl_temp.f524 = ct->f269;
  expl_temp.f523 = ct->f268;
  expl_temp.f522 = ct->f267;
  expl_temp.f521 = ct->f266;
  expl_temp.f520 = ct->f265;
  expl_temp.f519 = ct->f264;
  expl_temp.f518 = ct->f263;
  expl_temp.f517 = ct->f262;
  expl_temp.f516 = ct->f261;
  expl_temp.f515 = ct->f260;
  expl_temp.f514 = ct->f259;
  expl_temp.f513 = ct->f258;
  expl_temp.f512 = ct->f257;
  expl_temp.f511 = ct->f256;
  expl_temp.f510 = ct->f255;
  expl_temp.f509 = ct->f254;
  expl_temp.f508 = ct->f253;
  expl_temp.f507 = ct->f252;
  expl_temp.f506 = ct->f251;
  expl_temp.f505 = ct->f250;
  expl_temp.f504 = ct->f249;
  expl_temp.f503 = ct->f248;
  expl_temp.f502 = ct->f247;
  expl_temp.f501 = ct->f246;
  expl_temp.f500 = ct->f245;
  expl_temp.f499 = ct->f244;
  expl_temp.f498 = ct->f243;
  expl_temp.f497 = ct->f242;
  expl_temp.f496 = ct->f241;
  expl_temp.f495 = ct->f240;
  expl_temp.f494 = ct->f239;
  d_c_tmp = ct->f446 * ct->f481;
  ct_re_tmp = d_c_tmp * ct->f170.re;
  ct_im_tmp = d_c_tmp * ct->f170.im;
  d_c_tmp = ct->f517.re - ct->f553.re;
  b_ct_im_tmp = ct->f517.im - ct->f553.im;
  expl_temp.f493.re = -0.5 * (ct_re_tmp * d_c_tmp - ct_im_tmp * b_ct_im_tmp);
  expl_temp.f493.im = -0.5 * (ct_re_tmp * b_ct_im_tmp + ct_im_tmp * d_c_tmp);
  b_ct_re_tmp = ct->f437 * ct->f481;
  c_ct_re_tmp = b_ct_re_tmp * ct->f170.re;
  c_ct_im_tmp = b_ct_re_tmp * ct->f170.im;
  c_tmp = ct->f519.re - ct->f555.re;
  b_c_tmp = ct->f519.im - ct->f555.im;
  expl_temp.f492.re = -0.5 * (c_ct_re_tmp * c_tmp - c_ct_im_tmp * b_c_tmp);
  expl_temp.f492.im = -0.5 * (c_ct_re_tmp * b_c_tmp + c_ct_im_tmp * c_tmp);
  c_tmp = ct->f518.re - ct->f554.re;
  b_c_tmp = ct->f518.im - ct->f554.im;
  expl_temp.f491.re = -0.5 * (c_ct_re_tmp * c_tmp - c_ct_im_tmp * b_c_tmp);
  expl_temp.f491.im = -0.5 * (c_ct_re_tmp * b_c_tmp + c_ct_im_tmp * c_tmp);
  expl_temp.f490.re = -0.5 * (c_ct_re_tmp * d_c_tmp - c_ct_im_tmp * b_ct_im_tmp);
  expl_temp.f490.im = -0.5 * (c_ct_re_tmp * b_ct_im_tmp + c_ct_im_tmp * d_c_tmp);
  d_c_tmp = ct->f445 * ct->f479;
  b_ct_re_tmp = d_c_tmp * ct->f169.re;
  b_ct_im_tmp = d_c_tmp * ct->f169.im;
  d_c_tmp = ct->f515.re - ct->f552.re;
  d_ct_im_tmp = ct->f515.im - ct->f552.im;
  expl_temp.f489.re = -0.5 * (b_ct_re_tmp * d_c_tmp - b_ct_im_tmp * d_ct_im_tmp);
  expl_temp.f489.im = -0.5 * (b_ct_re_tmp * d_ct_im_tmp + b_ct_im_tmp * d_c_tmp);
  d_ct_re_tmp = ct->f514.re - ct->f551.re;
  e_ct_im_tmp = ct->f514.im - ct->f551.im;
  expl_temp.f488.re = -0.5 * (b_ct_re_tmp * d_ct_re_tmp - b_ct_im_tmp *
    e_ct_im_tmp);
  expl_temp.f488.im = -0.5 * (b_ct_re_tmp * e_ct_im_tmp + b_ct_im_tmp *
    d_ct_re_tmp);
  e_ct_re_tmp = ct->f513.re - ct->f550.re;
  f_ct_im_tmp = ct->f513.im - ct->f550.im;
  expl_temp.f487.re = -0.5 * (b_ct_re_tmp * e_ct_re_tmp - b_ct_im_tmp *
    f_ct_im_tmp);
  expl_temp.f487.im = -0.5 * (b_ct_re_tmp * f_ct_im_tmp + b_ct_im_tmp *
    e_ct_re_tmp);
  f_ct_re_tmp = ct->f436 * ct->f479;
  g_ct_re_tmp = f_ct_re_tmp * ct->f169.re;
  g_ct_im_tmp = f_ct_re_tmp * ct->f169.im;
  expl_temp.f486.re = -0.5 * (g_ct_re_tmp * d_c_tmp - g_ct_im_tmp * d_ct_im_tmp);
  expl_temp.f486.im = -0.5 * (g_ct_re_tmp * d_ct_im_tmp + g_ct_im_tmp * d_c_tmp);
  expl_temp.f485.re = -0.5 * (g_ct_re_tmp * d_ct_re_tmp - g_ct_im_tmp *
    e_ct_im_tmp);
  expl_temp.f485.im = -0.5 * (g_ct_re_tmp * e_ct_im_tmp + g_ct_im_tmp *
    d_ct_re_tmp);
  expl_temp.f484 = ct->f238;
  expl_temp.f483.re = -0.5 * (g_ct_re_tmp * e_ct_re_tmp - g_ct_im_tmp *
    f_ct_im_tmp);
  expl_temp.f483.im = -0.5 * (g_ct_re_tmp * f_ct_im_tmp + g_ct_im_tmp *
    e_ct_re_tmp);
  d_c_tmp = ct->f454 * ct->f498;
  d_ct_re_tmp = d_c_tmp * ct->f177.re;
  d_ct_im_tmp = d_c_tmp * ct->f177.im;
  d_c_tmp = ct->f508.re - ct->f549.re;
  e_ct_im_tmp = ct->f508.im - ct->f549.im;
  expl_temp.f482.re = -0.5 * (d_ct_re_tmp * d_c_tmp - d_ct_im_tmp * e_ct_im_tmp);
  expl_temp.f482.im = -0.5 * (d_ct_re_tmp * e_ct_im_tmp + d_ct_im_tmp * d_c_tmp);
  e_ct_re_tmp = ct->f453 * ct->f498;
  f_ct_re_tmp = e_ct_re_tmp * ct->f177.re;
  f_ct_im_tmp = e_ct_re_tmp * ct->f177.im;
  expl_temp.f481.re = -0.5 * (f_ct_re_tmp * d_c_tmp - f_ct_im_tmp * e_ct_im_tmp);
  expl_temp.f481.im = -0.5 * (f_ct_re_tmp * e_ct_im_tmp + f_ct_im_tmp * d_c_tmp);
  d_c_tmp = ct->f452 * ct->f496;
  e_ct_re_tmp = d_c_tmp * ct->f176.re;
  e_ct_im_tmp = d_c_tmp * ct->f176.im;
  d_c_tmp = ct->f507.re - ct->f548.re;
  h_ct_im_tmp = ct->f507.im - ct->f548.im;
  expl_temp.f480.re = -0.5 * (e_ct_re_tmp * d_c_tmp - e_ct_im_tmp * h_ct_im_tmp);
  expl_temp.f480.im = -0.5 * (e_ct_re_tmp * h_ct_im_tmp + e_ct_im_tmp * d_c_tmp);
  h_ct_re_tmp = ct->f443 * ct->f496;
  i_ct_re_tmp = h_ct_re_tmp * ct->f176.re;
  i_ct_im_tmp = h_ct_re_tmp * ct->f176.im;
  expl_temp.f479.re = -0.5 * (i_ct_re_tmp * d_c_tmp - i_ct_im_tmp * h_ct_im_tmp);
  expl_temp.f479.im = -0.5 * (i_ct_re_tmp * h_ct_im_tmp + i_ct_im_tmp * d_c_tmp);
  d_c_tmp = ct->f451 * ct->f493;
  h_ct_re_tmp = d_c_tmp * ct->f175.re;
  h_ct_im_tmp = d_c_tmp * ct->f175.im;
  d_c_tmp = ct->f505.re - ct->f547.re;
  j_ct_im_tmp = ct->f505.im - ct->f547.im;
  expl_temp.f478.re = -0.5 * (h_ct_re_tmp * d_c_tmp - h_ct_im_tmp * j_ct_im_tmp);
  expl_temp.f478.im = -0.5 * (h_ct_re_tmp * j_ct_im_tmp + h_ct_im_tmp * d_c_tmp);
  j_ct_re_tmp = ct->f442 * ct->f493;
  k_ct_re_tmp = j_ct_re_tmp * ct->f175.re;
  k_ct_im_tmp = j_ct_re_tmp * ct->f175.im;
  expl_temp.f477.re = -0.5 * (k_ct_re_tmp * d_c_tmp - k_ct_im_tmp * j_ct_im_tmp);
  expl_temp.f477.im = -0.5 * (k_ct_re_tmp * j_ct_im_tmp + k_ct_im_tmp * d_c_tmp);
  d_c_tmp = ct->f450 * ct->f491;
  j_ct_re_tmp = d_c_tmp * ct->f174.re;
  j_ct_im_tmp = d_c_tmp * ct->f174.im;
  d_c_tmp = ct->f504.re - ct->f546.re;
  l_ct_im_tmp = ct->f504.im - ct->f546.im;
  expl_temp.f476.re = -0.5 * (j_ct_re_tmp * d_c_tmp - j_ct_im_tmp * l_ct_im_tmp);
  expl_temp.f476.im = -0.5 * (j_ct_re_tmp * l_ct_im_tmp + j_ct_im_tmp * d_c_tmp);
  l_ct_re_tmp = ct->f441 * ct->f491;
  m_ct_re_tmp = l_ct_re_tmp * ct->f174.re;
  m_ct_im_tmp = l_ct_re_tmp * ct->f174.im;
  expl_temp.f475.re = -0.5 * (m_ct_re_tmp * d_c_tmp - m_ct_im_tmp * l_ct_im_tmp);
  expl_temp.f475.im = -0.5 * (m_ct_re_tmp * l_ct_im_tmp + m_ct_im_tmp * d_c_tmp);
  d_c_tmp = ct->f449 * ct->f489;
  l_ct_re_tmp = d_c_tmp * ct->f173.re;
  l_ct_im_tmp = d_c_tmp * ct->f173.im;
  d_c_tmp = ct->f503.re - ct->f545.re;
  n_ct_im_tmp = ct->f503.im - ct->f545.im;
  expl_temp.f474.re = -0.5 * (l_ct_re_tmp * d_c_tmp - l_ct_im_tmp * n_ct_im_tmp);
  expl_temp.f474.im = -0.5 * (l_ct_re_tmp * n_ct_im_tmp + l_ct_im_tmp * d_c_tmp);
  expl_temp.f473 = ct->f237;
  n_ct_re_tmp = ct->f440 * ct->f489;
  o_ct_re_tmp = n_ct_re_tmp * ct->f173.re;
  o_ct_im_tmp = n_ct_re_tmp * ct->f173.im;
  expl_temp.f472.re = -0.5 * (o_ct_re_tmp * d_c_tmp - o_ct_im_tmp * n_ct_im_tmp);
  expl_temp.f472.im = -0.5 * (o_ct_re_tmp * n_ct_im_tmp + o_ct_im_tmp * d_c_tmp);
  d_c_tmp = ct->f448 * ct->f487;
  n_ct_re_tmp = d_c_tmp * ct->f172.re;
  n_ct_im_tmp = d_c_tmp * ct->f172.im;
  d_c_tmp = ct->f502.re - ct->f544.re;
  p_ct_im_tmp = ct->f502.im - ct->f544.im;
  expl_temp.f471.re = -0.5 * (n_ct_re_tmp * d_c_tmp - n_ct_im_tmp * p_ct_im_tmp);
  expl_temp.f471.im = -0.5 * (n_ct_re_tmp * p_ct_im_tmp + n_ct_im_tmp * d_c_tmp);
  p_ct_re_tmp = ct->f439 * ct->f487;
  q_ct_re_tmp = p_ct_re_tmp * ct->f172.re;
  q_ct_im_tmp = p_ct_re_tmp * ct->f172.im;
  expl_temp.f470.re = -0.5 * (q_ct_re_tmp * d_c_tmp - q_ct_im_tmp * p_ct_im_tmp);
  expl_temp.f470.im = -0.5 * (q_ct_re_tmp * p_ct_im_tmp + q_ct_im_tmp * d_c_tmp);
  d_c_tmp = ct->f447 * ct->f485;
  p_ct_re_tmp = d_c_tmp * ct->f171.re;
  p_ct_im_tmp = d_c_tmp * ct->f171.im;
  d_c_tmp = ct->f501.re - ct->f543.re;
  b_c_tmp = ct->f501.im - ct->f543.im;
  expl_temp.f469.re = -0.5 * (p_ct_re_tmp * d_c_tmp - p_ct_im_tmp * b_c_tmp);
  expl_temp.f469.im = -0.5 * (p_ct_re_tmp * b_c_tmp + p_ct_im_tmp * d_c_tmp);
  c_tmp = ct->f438 * ct->f485;
  r_ct_re_tmp = c_tmp * ct->f171.re;
  c_c_tmp = c_tmp * ct->f171.im;
  expl_temp.f468.re = -0.5 * (r_ct_re_tmp * d_c_tmp - c_c_tmp * b_c_tmp);
  expl_temp.f468.im = -0.5 * (r_ct_re_tmp * b_c_tmp + c_c_tmp * d_c_tmp);
  d_c_tmp = ct->f500.re - ct->f542.re;
  b_c_tmp = ct->f500.im - ct->f542.im;
  expl_temp.f467.re = -0.5 * (ct_re_tmp * d_c_tmp - ct_im_tmp * b_c_tmp);
  expl_temp.f467.im = -0.5 * (ct_re_tmp * b_c_tmp + ct_im_tmp * d_c_tmp);
  expl_temp.f466.re = -0.5 * (c_ct_re_tmp * d_c_tmp - c_ct_im_tmp * b_c_tmp);
  expl_temp.f466.im = -0.5 * (c_ct_re_tmp * b_c_tmp + c_ct_im_tmp * d_c_tmp);
  d_c_tmp = ct->f499.re - ct->f541.re;
  b_c_tmp = ct->f499.im - ct->f541.im;
  expl_temp.f465.re = -0.5 * (b_ct_re_tmp * d_c_tmp - b_ct_im_tmp * b_c_tmp);
  expl_temp.f465.im = -0.5 * (b_ct_re_tmp * b_c_tmp + b_ct_im_tmp * d_c_tmp);
  expl_temp.f464.re = -0.5 * (g_ct_re_tmp * d_c_tmp - g_ct_im_tmp * b_c_tmp);
  expl_temp.f464.im = -0.5 * (g_ct_re_tmp * b_c_tmp + g_ct_im_tmp * d_c_tmp);
  c_tmp = d_ct_re_tmp * ct->f218.re - d_ct_im_tmp * ct->f218.im;
  b_c_tmp = d_ct_re_tmp * ct->f218.im + d_ct_im_tmp * ct->f218.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f463.re = c_tmp / 2.0;
    expl_temp.f463.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f463.re = 0.0;
    expl_temp.f463.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f463.re = c_tmp / 2.0;
    expl_temp.f463.im = b_c_tmp / 2.0;
  }

  expl_temp.f462 = ct->f236;
  c_tmp = d_ct_re_tmp * ct->f217.re - d_ct_im_tmp * ct->f217.im;
  b_c_tmp = d_ct_re_tmp * ct->f217.im + d_ct_im_tmp * ct->f217.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f461.re = c_tmp / 2.0;
    expl_temp.f461.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f461.re = 0.0;
    expl_temp.f461.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f461.re = c_tmp / 2.0;
    expl_temp.f461.im = b_c_tmp / 2.0;
  }

  c_tmp = d_ct_re_tmp * ct->f216.re - d_ct_im_tmp * ct->f216.im;
  b_c_tmp = d_ct_re_tmp * ct->f216.im + d_ct_im_tmp * ct->f216.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f460.re = c_tmp / 2.0;
    expl_temp.f460.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f460.re = 0.0;
    expl_temp.f460.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f460.re = c_tmp / 2.0;
    expl_temp.f460.im = b_c_tmp / 2.0;
  }

  c_tmp = f_ct_re_tmp * ct->f218.re - f_ct_im_tmp * ct->f218.im;
  b_c_tmp = f_ct_re_tmp * ct->f218.im + f_ct_im_tmp * ct->f218.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f459.re = c_tmp / 2.0;
    expl_temp.f459.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f459.re = 0.0;
    expl_temp.f459.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f459.re = c_tmp / 2.0;
    expl_temp.f459.im = b_c_tmp / 2.0;
  }

  c_tmp = f_ct_re_tmp * ct->f217.re - f_ct_im_tmp * ct->f217.im;
  b_c_tmp = f_ct_re_tmp * ct->f217.im + f_ct_im_tmp * ct->f217.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f458.re = c_tmp / 2.0;
    expl_temp.f458.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f458.re = 0.0;
    expl_temp.f458.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f458.re = c_tmp / 2.0;
    expl_temp.f458.im = b_c_tmp / 2.0;
  }

  c_tmp = f_ct_re_tmp * ct->f216.re - f_ct_im_tmp * ct->f216.im;
  b_c_tmp = f_ct_re_tmp * ct->f216.im + f_ct_im_tmp * ct->f216.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f457.re = c_tmp / 2.0;
    expl_temp.f457.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f457.re = 0.0;
    expl_temp.f457.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f457.re = c_tmp / 2.0;
    expl_temp.f457.im = b_c_tmp / 2.0;
  }

  c_tmp = e_ct_re_tmp * ct->f215.re - e_ct_im_tmp * ct->f215.im;
  b_c_tmp = e_ct_re_tmp * ct->f215.im + e_ct_im_tmp * ct->f215.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f456.re = c_tmp / 2.0;
    expl_temp.f456.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f456.re = 0.0;
    expl_temp.f456.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f456.re = c_tmp / 2.0;
    expl_temp.f456.im = b_c_tmp / 2.0;
  }

  c_tmp = e_ct_re_tmp * ct->f214.re - e_ct_im_tmp * ct->f214.im;
  b_c_tmp = e_ct_re_tmp * ct->f214.im + e_ct_im_tmp * ct->f214.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f455.re = c_tmp / 2.0;
    expl_temp.f455.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f455.re = 0.0;
    expl_temp.f455.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f455.re = c_tmp / 2.0;
    expl_temp.f455.im = b_c_tmp / 2.0;
  }

  c_tmp = e_ct_re_tmp * ct->f212.re - e_ct_im_tmp * ct->f212.im;
  b_c_tmp = e_ct_re_tmp * ct->f212.im + e_ct_im_tmp * ct->f212.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f454.re = c_tmp / 2.0;
    expl_temp.f454.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f454.re = 0.0;
    expl_temp.f454.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f454.re = c_tmp / 2.0;
    expl_temp.f454.im = b_c_tmp / 2.0;
  }

  c_tmp = i_ct_re_tmp * ct->f215.re - i_ct_im_tmp * ct->f215.im;
  b_c_tmp = i_ct_re_tmp * ct->f215.im + i_ct_im_tmp * ct->f215.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f453.re = c_tmp / 2.0;
    expl_temp.f453.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f453.re = 0.0;
    expl_temp.f453.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f453.re = c_tmp / 2.0;
    expl_temp.f453.im = b_c_tmp / 2.0;
  }

  c_tmp = i_ct_re_tmp * ct->f214.re - i_ct_im_tmp * ct->f214.im;
  b_c_tmp = i_ct_re_tmp * ct->f214.im + i_ct_im_tmp * ct->f214.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f452.re = c_tmp / 2.0;
    expl_temp.f452.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f452.re = 0.0;
    expl_temp.f452.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f452.re = c_tmp / 2.0;
    expl_temp.f452.im = b_c_tmp / 2.0;
  }

  expl_temp.f451 = ct->f235;
  c_tmp = i_ct_re_tmp * ct->f212.re - i_ct_im_tmp * ct->f212.im;
  b_c_tmp = i_ct_re_tmp * ct->f212.im + i_ct_im_tmp * ct->f212.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f450.re = c_tmp / 2.0;
    expl_temp.f450.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f450.re = 0.0;
    expl_temp.f450.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f450.re = c_tmp / 2.0;
    expl_temp.f450.im = b_c_tmp / 2.0;
  }

  c_tmp = h_ct_re_tmp * ct->f211.re - h_ct_im_tmp * ct->f211.im;
  b_c_tmp = h_ct_re_tmp * ct->f211.im + h_ct_im_tmp * ct->f211.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f449.re = c_tmp / 2.0;
    expl_temp.f449.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f449.re = 0.0;
    expl_temp.f449.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f449.re = c_tmp / 2.0;
    expl_temp.f449.im = b_c_tmp / 2.0;
  }

  c_tmp = h_ct_re_tmp * ct->f210.re - h_ct_im_tmp * ct->f210.im;
  b_c_tmp = h_ct_re_tmp * ct->f210.im + h_ct_im_tmp * ct->f210.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f448.re = c_tmp / 2.0;
    expl_temp.f448.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f448.re = 0.0;
    expl_temp.f448.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f448.re = c_tmp / 2.0;
    expl_temp.f448.im = b_c_tmp / 2.0;
  }

  c_tmp = h_ct_re_tmp * ct->f209.re - h_ct_im_tmp * ct->f209.im;
  b_c_tmp = h_ct_re_tmp * ct->f209.im + h_ct_im_tmp * ct->f209.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f447.re = c_tmp / 2.0;
    expl_temp.f447.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f447.re = 0.0;
    expl_temp.f447.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f447.re = c_tmp / 2.0;
    expl_temp.f447.im = b_c_tmp / 2.0;
  }

  c_tmp = k_ct_re_tmp * ct->f211.re - k_ct_im_tmp * ct->f211.im;
  b_c_tmp = k_ct_re_tmp * ct->f211.im + k_ct_im_tmp * ct->f211.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f446.re = c_tmp / 2.0;
    expl_temp.f446.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f446.re = 0.0;
    expl_temp.f446.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f446.re = c_tmp / 2.0;
    expl_temp.f446.im = b_c_tmp / 2.0;
  }

  c_tmp = k_ct_re_tmp * ct->f210.re - k_ct_im_tmp * ct->f210.im;
  b_c_tmp = k_ct_re_tmp * ct->f210.im + k_ct_im_tmp * ct->f210.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f445.re = c_tmp / 2.0;
    expl_temp.f445.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f445.re = 0.0;
    expl_temp.f445.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f445.re = c_tmp / 2.0;
    expl_temp.f445.im = b_c_tmp / 2.0;
  }

  c_tmp = k_ct_re_tmp * ct->f209.re - k_ct_im_tmp * ct->f209.im;
  b_c_tmp = k_ct_re_tmp * ct->f209.im + k_ct_im_tmp * ct->f209.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f444.re = c_tmp / 2.0;
    expl_temp.f444.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f444.re = 0.0;
    expl_temp.f444.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f444.re = c_tmp / 2.0;
    expl_temp.f444.im = b_c_tmp / 2.0;
  }

  c_tmp = j_ct_re_tmp * ct->f208.re - j_ct_im_tmp * ct->f208.im;
  b_c_tmp = j_ct_re_tmp * ct->f208.im + j_ct_im_tmp * ct->f208.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f443.re = c_tmp / 2.0;
    expl_temp.f443.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f443.re = 0.0;
    expl_temp.f443.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f443.re = c_tmp / 2.0;
    expl_temp.f443.im = b_c_tmp / 2.0;
  }

  c_tmp = j_ct_re_tmp * ct->f207.re - j_ct_im_tmp * ct->f207.im;
  b_c_tmp = j_ct_re_tmp * ct->f207.im + j_ct_im_tmp * ct->f207.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f442.re = c_tmp / 2.0;
    expl_temp.f442.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f442.re = 0.0;
    expl_temp.f442.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f442.re = c_tmp / 2.0;
    expl_temp.f442.im = b_c_tmp / 2.0;
  }

  c_tmp = j_ct_re_tmp * ct->f206.re - j_ct_im_tmp * ct->f206.im;
  b_c_tmp = j_ct_re_tmp * ct->f206.im + j_ct_im_tmp * ct->f206.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f441.re = c_tmp / 2.0;
    expl_temp.f441.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f441.re = 0.0;
    expl_temp.f441.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f441.re = c_tmp / 2.0;
    expl_temp.f441.im = b_c_tmp / 2.0;
  }

  expl_temp.f440 = ct->f234;
  c_tmp = m_ct_re_tmp * ct->f208.re - m_ct_im_tmp * ct->f208.im;
  b_c_tmp = m_ct_re_tmp * ct->f208.im + m_ct_im_tmp * ct->f208.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f439.re = c_tmp / 2.0;
    expl_temp.f439.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f439.re = 0.0;
    expl_temp.f439.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f439.re = c_tmp / 2.0;
    expl_temp.f439.im = b_c_tmp / 2.0;
  }

  c_tmp = m_ct_re_tmp * ct->f207.re - m_ct_im_tmp * ct->f207.im;
  b_c_tmp = m_ct_re_tmp * ct->f207.im + m_ct_im_tmp * ct->f207.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f438.re = c_tmp / 2.0;
    expl_temp.f438.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f438.re = 0.0;
    expl_temp.f438.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f438.re = c_tmp / 2.0;
    expl_temp.f438.im = b_c_tmp / 2.0;
  }

  c_tmp = m_ct_re_tmp * ct->f206.re - m_ct_im_tmp * ct->f206.im;
  b_c_tmp = m_ct_re_tmp * ct->f206.im + m_ct_im_tmp * ct->f206.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f437.re = c_tmp / 2.0;
    expl_temp.f437.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f437.re = 0.0;
    expl_temp.f437.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f437.re = c_tmp / 2.0;
    expl_temp.f437.im = b_c_tmp / 2.0;
  }

  c_tmp = l_ct_re_tmp * ct->f205.re - l_ct_im_tmp * ct->f205.im;
  b_c_tmp = l_ct_re_tmp * ct->f205.im + l_ct_im_tmp * ct->f205.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f436.re = c_tmp / 2.0;
    expl_temp.f436.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f436.re = 0.0;
    expl_temp.f436.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f436.re = c_tmp / 2.0;
    expl_temp.f436.im = b_c_tmp / 2.0;
  }

  c_tmp = l_ct_re_tmp * ct->f204.re - l_ct_im_tmp * ct->f204.im;
  b_c_tmp = l_ct_re_tmp * ct->f204.im + l_ct_im_tmp * ct->f204.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f435.re = c_tmp / 2.0;
    expl_temp.f435.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f435.re = 0.0;
    expl_temp.f435.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f435.re = c_tmp / 2.0;
    expl_temp.f435.im = b_c_tmp / 2.0;
  }

  c_tmp = l_ct_re_tmp * ct->f203.re - l_ct_im_tmp * ct->f203.im;
  b_c_tmp = l_ct_re_tmp * ct->f203.im + l_ct_im_tmp * ct->f203.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f434.re = c_tmp / 2.0;
    expl_temp.f434.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f434.re = 0.0;
    expl_temp.f434.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f434.re = c_tmp / 2.0;
    expl_temp.f434.im = b_c_tmp / 2.0;
  }

  c_tmp = o_ct_re_tmp * ct->f205.re - o_ct_im_tmp * ct->f205.im;
  b_c_tmp = o_ct_re_tmp * ct->f205.im + o_ct_im_tmp * ct->f205.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f433.re = c_tmp / 2.0;
    expl_temp.f433.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f433.re = 0.0;
    expl_temp.f433.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f433.re = c_tmp / 2.0;
    expl_temp.f433.im = b_c_tmp / 2.0;
  }

  c_tmp = o_ct_re_tmp * ct->f204.re - o_ct_im_tmp * ct->f204.im;
  b_c_tmp = o_ct_re_tmp * ct->f204.im + o_ct_im_tmp * ct->f204.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f432.re = c_tmp / 2.0;
    expl_temp.f432.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f432.re = 0.0;
    expl_temp.f432.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f432.re = c_tmp / 2.0;
    expl_temp.f432.im = b_c_tmp / 2.0;
  }

  c_tmp = o_ct_re_tmp * ct->f203.re - o_ct_im_tmp * ct->f203.im;
  b_c_tmp = o_ct_re_tmp * ct->f203.im + o_ct_im_tmp * ct->f203.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f431.re = c_tmp / 2.0;
    expl_temp.f431.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f431.re = 0.0;
    expl_temp.f431.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f431.re = c_tmp / 2.0;
    expl_temp.f431.im = b_c_tmp / 2.0;
  }

  c_tmp = n_ct_re_tmp * ct->f200.re - n_ct_im_tmp * ct->f200.im;
  b_c_tmp = n_ct_re_tmp * ct->f200.im + n_ct_im_tmp * ct->f200.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f430.re = c_tmp / 2.0;
    expl_temp.f430.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f430.re = 0.0;
    expl_temp.f430.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f430.re = c_tmp / 2.0;
    expl_temp.f430.im = b_c_tmp / 2.0;
  }

  expl_temp.f429 = ct->f233;
  expl_temp.f428 = ct->f232;
  c_tmp = n_ct_re_tmp * ct->f199.re - n_ct_im_tmp * ct->f199.im;
  b_c_tmp = n_ct_re_tmp * ct->f199.im + n_ct_im_tmp * ct->f199.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f427.re = c_tmp / 2.0;
    expl_temp.f427.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f427.re = 0.0;
    expl_temp.f427.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f427.re = c_tmp / 2.0;
    expl_temp.f427.im = b_c_tmp / 2.0;
  }

  c_tmp = n_ct_re_tmp * ct->f198.re - n_ct_im_tmp * ct->f198.im;
  b_c_tmp = n_ct_re_tmp * ct->f198.im + n_ct_im_tmp * ct->f198.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f426.re = c_tmp / 2.0;
    expl_temp.f426.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f426.re = 0.0;
    expl_temp.f426.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f426.re = c_tmp / 2.0;
    expl_temp.f426.im = b_c_tmp / 2.0;
  }

  c_tmp = q_ct_re_tmp * ct->f200.re - q_ct_im_tmp * ct->f200.im;
  b_c_tmp = q_ct_re_tmp * ct->f200.im + q_ct_im_tmp * ct->f200.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f425.re = c_tmp / 2.0;
    expl_temp.f425.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f425.re = 0.0;
    expl_temp.f425.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f425.re = c_tmp / 2.0;
    expl_temp.f425.im = b_c_tmp / 2.0;
  }

  c_tmp = q_ct_re_tmp * ct->f199.re - q_ct_im_tmp * ct->f199.im;
  b_c_tmp = q_ct_re_tmp * ct->f199.im + q_ct_im_tmp * ct->f199.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f424.re = c_tmp / 2.0;
    expl_temp.f424.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f424.re = 0.0;
    expl_temp.f424.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f424.re = c_tmp / 2.0;
    expl_temp.f424.im = b_c_tmp / 2.0;
  }

  c_tmp = q_ct_re_tmp * ct->f198.re - q_ct_im_tmp * ct->f198.im;
  b_c_tmp = q_ct_re_tmp * ct->f198.im + q_ct_im_tmp * ct->f198.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f423.re = c_tmp / 2.0;
    expl_temp.f423.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f423.re = 0.0;
    expl_temp.f423.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f423.re = c_tmp / 2.0;
    expl_temp.f423.im = b_c_tmp / 2.0;
  }

  c_tmp = p_ct_re_tmp * ct->f197.re - p_ct_im_tmp * ct->f197.im;
  b_c_tmp = p_ct_re_tmp * ct->f197.im + p_ct_im_tmp * ct->f197.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f422.re = c_tmp / 2.0;
    expl_temp.f422.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f422.re = 0.0;
    expl_temp.f422.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f422.re = c_tmp / 2.0;
    expl_temp.f422.im = b_c_tmp / 2.0;
  }

  c_tmp = p_ct_re_tmp * ct->f196.re - p_ct_im_tmp * ct->f196.im;
  b_c_tmp = p_ct_re_tmp * ct->f196.im + p_ct_im_tmp * ct->f196.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f421.re = c_tmp / 2.0;
    expl_temp.f421.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f421.re = 0.0;
    expl_temp.f421.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f421.re = c_tmp / 2.0;
    expl_temp.f421.im = b_c_tmp / 2.0;
  }

  c_tmp = p_ct_re_tmp * ct->f195.re - p_ct_im_tmp * ct->f195.im;
  b_c_tmp = p_ct_re_tmp * ct->f195.im + p_ct_im_tmp * ct->f195.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f420.re = c_tmp / 2.0;
    expl_temp.f420.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f420.re = 0.0;
    expl_temp.f420.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f420.re = c_tmp / 2.0;
    expl_temp.f420.im = b_c_tmp / 2.0;
  }

  c_tmp = r_ct_re_tmp * ct->f197.re - c_c_tmp * ct->f197.im;
  b_c_tmp = r_ct_re_tmp * ct->f197.im + c_c_tmp * ct->f197.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f419.re = c_tmp / 2.0;
    expl_temp.f419.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f419.re = 0.0;
    expl_temp.f419.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f419.re = c_tmp / 2.0;
    expl_temp.f419.im = b_c_tmp / 2.0;
  }

  c_tmp = r_ct_re_tmp * ct->f196.re - c_c_tmp * ct->f196.im;
  b_c_tmp = r_ct_re_tmp * ct->f196.im + c_c_tmp * ct->f196.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f418.re = c_tmp / 2.0;
    expl_temp.f418.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f418.re = 0.0;
    expl_temp.f418.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f418.re = c_tmp / 2.0;
    expl_temp.f418.im = b_c_tmp / 2.0;
  }

  c_tmp = r_ct_re_tmp * ct->f195.re - c_c_tmp * ct->f195.im;
  b_c_tmp = r_ct_re_tmp * ct->f195.im + c_c_tmp * ct->f195.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f417.re = c_tmp / 2.0;
    expl_temp.f417.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f417.re = 0.0;
    expl_temp.f417.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f417.re = c_tmp / 2.0;
    expl_temp.f417.im = b_c_tmp / 2.0;
  }

  c_tmp = ct_re_tmp * ct->f194.re - ct_im_tmp * ct->f194.im;
  b_c_tmp = ct_re_tmp * ct->f194.im + ct_im_tmp * ct->f194.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f416.re = c_tmp / 2.0;
    expl_temp.f416.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f416.re = 0.0;
    expl_temp.f416.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f416.re = c_tmp / 2.0;
    expl_temp.f416.im = b_c_tmp / 2.0;
  }

  c_tmp = ct_re_tmp * ct->f193.re - ct_im_tmp * ct->f193.im;
  b_c_tmp = ct_re_tmp * ct->f193.im + ct_im_tmp * ct->f193.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f415.re = c_tmp / 2.0;
    expl_temp.f415.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f415.re = 0.0;
    expl_temp.f415.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f415.re = c_tmp / 2.0;
    expl_temp.f415.im = b_c_tmp / 2.0;
  }

  c_tmp = ct_re_tmp * ct->f192.re - ct_im_tmp * ct->f192.im;
  b_c_tmp = ct_re_tmp * ct->f192.im + ct_im_tmp * ct->f192.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f414.re = c_tmp / 2.0;
    expl_temp.f414.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f414.re = 0.0;
    expl_temp.f414.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f414.re = c_tmp / 2.0;
    expl_temp.f414.im = b_c_tmp / 2.0;
  }

  c_tmp = c_ct_re_tmp * ct->f194.re - c_ct_im_tmp * ct->f194.im;
  b_c_tmp = c_ct_re_tmp * ct->f194.im + c_ct_im_tmp * ct->f194.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f413.re = c_tmp / 2.0;
    expl_temp.f413.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f413.re = 0.0;
    expl_temp.f413.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f413.re = c_tmp / 2.0;
    expl_temp.f413.im = b_c_tmp / 2.0;
  }

  c_tmp = c_ct_re_tmp * ct->f193.re - c_ct_im_tmp * ct->f193.im;
  b_c_tmp = c_ct_re_tmp * ct->f193.im + c_ct_im_tmp * ct->f193.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f412.re = c_tmp / 2.0;
    expl_temp.f412.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f412.re = 0.0;
    expl_temp.f412.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f412.re = c_tmp / 2.0;
    expl_temp.f412.im = b_c_tmp / 2.0;
  }

  c_tmp = c_ct_re_tmp * ct->f192.re - c_ct_im_tmp * ct->f192.im;
  b_c_tmp = c_ct_re_tmp * ct->f192.im + c_ct_im_tmp * ct->f192.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f411.re = c_tmp / 2.0;
    expl_temp.f411.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f411.re = 0.0;
    expl_temp.f411.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f411.re = c_tmp / 2.0;
    expl_temp.f411.im = b_c_tmp / 2.0;
  }

  c_tmp = b_ct_re_tmp * ct->f191.re - b_ct_im_tmp * ct->f191.im;
  b_c_tmp = b_ct_re_tmp * ct->f191.im + b_ct_im_tmp * ct->f191.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f410.re = c_tmp / 2.0;
    expl_temp.f410.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f410.re = 0.0;
    expl_temp.f410.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f410.re = c_tmp / 2.0;
    expl_temp.f410.im = b_c_tmp / 2.0;
  }

  c_tmp = b_ct_re_tmp * ct->f189.re - b_ct_im_tmp * ct->f189.im;
  b_c_tmp = b_ct_re_tmp * ct->f189.im + b_ct_im_tmp * ct->f189.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f409.re = c_tmp / 2.0;
    expl_temp.f409.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f409.re = 0.0;
    expl_temp.f409.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f409.re = c_tmp / 2.0;
    expl_temp.f409.im = b_c_tmp / 2.0;
  }

  c_tmp = b_ct_re_tmp * ct->f188.re - b_ct_im_tmp * ct->f188.im;
  b_c_tmp = b_ct_re_tmp * ct->f188.im + b_ct_im_tmp * ct->f188.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f408.re = c_tmp / 2.0;
    expl_temp.f408.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f408.re = 0.0;
    expl_temp.f408.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f408.re = c_tmp / 2.0;
    expl_temp.f408.im = b_c_tmp / 2.0;
  }

  c_tmp = g_ct_re_tmp * ct->f191.re - g_ct_im_tmp * ct->f191.im;
  b_c_tmp = g_ct_re_tmp * ct->f191.im + g_ct_im_tmp * ct->f191.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f407.re = c_tmp / 2.0;
    expl_temp.f407.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f407.re = 0.0;
    expl_temp.f407.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f407.re = c_tmp / 2.0;
    expl_temp.f407.im = b_c_tmp / 2.0;
  }

  c_tmp = g_ct_re_tmp * ct->f189.re - g_ct_im_tmp * ct->f189.im;
  b_c_tmp = g_ct_re_tmp * ct->f189.im + g_ct_im_tmp * ct->f189.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f406.re = c_tmp / 2.0;
    expl_temp.f406.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f406.re = 0.0;
    expl_temp.f406.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f406.re = c_tmp / 2.0;
    expl_temp.f406.im = b_c_tmp / 2.0;
  }

  c_tmp = g_ct_re_tmp * ct->f188.re - g_ct_im_tmp * ct->f188.im;
  b_c_tmp = g_ct_re_tmp * ct->f188.im + g_ct_im_tmp * ct->f188.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f405.re = c_tmp / 2.0;
    expl_temp.f405.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f405.re = 0.0;
    expl_temp.f405.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f405.re = c_tmp / 2.0;
    expl_temp.f405.im = b_c_tmp / 2.0;
  }

  c_tmp = d_ct_re_tmp * ct->f187.re - d_ct_im_tmp * ct->f187.im;
  b_c_tmp = d_ct_re_tmp * ct->f187.im + d_ct_im_tmp * ct->f187.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f404.re = c_tmp / 2.0;
    expl_temp.f404.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f404.re = 0.0;
    expl_temp.f404.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f404.re = c_tmp / 2.0;
    expl_temp.f404.im = b_c_tmp / 2.0;
  }

  c_tmp = f_ct_re_tmp * ct->f187.re - f_ct_im_tmp * ct->f187.im;
  b_c_tmp = f_ct_re_tmp * ct->f187.im + f_ct_im_tmp * ct->f187.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f403.re = c_tmp / 2.0;
    expl_temp.f403.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f403.re = 0.0;
    expl_temp.f403.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f403.re = c_tmp / 2.0;
    expl_temp.f403.im = b_c_tmp / 2.0;
  }

  c_tmp = e_ct_re_tmp * ct->f186.re - e_ct_im_tmp * ct->f186.im;
  b_c_tmp = e_ct_re_tmp * ct->f186.im + e_ct_im_tmp * ct->f186.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f402.re = c_tmp / 2.0;
    expl_temp.f402.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f402.re = 0.0;
    expl_temp.f402.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f402.re = c_tmp / 2.0;
    expl_temp.f402.im = b_c_tmp / 2.0;
  }

  c_tmp = i_ct_re_tmp * ct->f186.re - i_ct_im_tmp * ct->f186.im;
  b_c_tmp = i_ct_re_tmp * ct->f186.im + i_ct_im_tmp * ct->f186.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f401.re = c_tmp / 2.0;
    expl_temp.f401.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f401.re = 0.0;
    expl_temp.f401.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f401.re = c_tmp / 2.0;
    expl_temp.f401.im = b_c_tmp / 2.0;
  }

  c_tmp = h_ct_re_tmp * ct->f185.re - h_ct_im_tmp * ct->f185.im;
  b_c_tmp = h_ct_re_tmp * ct->f185.im + h_ct_im_tmp * ct->f185.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f400.re = c_tmp / 2.0;
    expl_temp.f400.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f400.re = 0.0;
    expl_temp.f400.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f400.re = c_tmp / 2.0;
    expl_temp.f400.im = b_c_tmp / 2.0;
  }

  c_tmp = k_ct_re_tmp * ct->f185.re - k_ct_im_tmp * ct->f185.im;
  b_c_tmp = k_ct_re_tmp * ct->f185.im + k_ct_im_tmp * ct->f185.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f399.re = c_tmp / 2.0;
    expl_temp.f399.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f399.re = 0.0;
    expl_temp.f399.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f399.re = c_tmp / 2.0;
    expl_temp.f399.im = b_c_tmp / 2.0;
  }

  c_tmp = j_ct_re_tmp * ct->f184.re - j_ct_im_tmp * ct->f184.im;
  b_c_tmp = j_ct_re_tmp * ct->f184.im + j_ct_im_tmp * ct->f184.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f398.re = c_tmp / 2.0;
    expl_temp.f398.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f398.re = 0.0;
    expl_temp.f398.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f398.re = c_tmp / 2.0;
    expl_temp.f398.im = b_c_tmp / 2.0;
  }

  c_tmp = m_ct_re_tmp * ct->f184.re - m_ct_im_tmp * ct->f184.im;
  b_c_tmp = m_ct_re_tmp * ct->f184.im + m_ct_im_tmp * ct->f184.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f397.re = c_tmp / 2.0;
    expl_temp.f397.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f397.re = 0.0;
    expl_temp.f397.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f397.re = c_tmp / 2.0;
    expl_temp.f397.im = b_c_tmp / 2.0;
  }

  c_tmp = l_ct_re_tmp * ct->f183.re - l_ct_im_tmp * ct->f183.im;
  b_c_tmp = l_ct_re_tmp * ct->f183.im + l_ct_im_tmp * ct->f183.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f396.re = c_tmp / 2.0;
    expl_temp.f396.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f396.re = 0.0;
    expl_temp.f396.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f396.re = c_tmp / 2.0;
    expl_temp.f396.im = b_c_tmp / 2.0;
  }

  c_tmp = o_ct_re_tmp * ct->f183.re - o_ct_im_tmp * ct->f183.im;
  b_c_tmp = o_ct_re_tmp * ct->f183.im + o_ct_im_tmp * ct->f183.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f395.re = c_tmp / 2.0;
    expl_temp.f395.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f395.re = 0.0;
    expl_temp.f395.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f395.re = c_tmp / 2.0;
    expl_temp.f395.im = b_c_tmp / 2.0;
  }

  c_tmp = n_ct_re_tmp * ct->f182.re - n_ct_im_tmp * ct->f182.im;
  b_c_tmp = n_ct_re_tmp * ct->f182.im + n_ct_im_tmp * ct->f182.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f394.re = c_tmp / 2.0;
    expl_temp.f394.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f394.re = 0.0;
    expl_temp.f394.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f394.re = c_tmp / 2.0;
    expl_temp.f394.im = b_c_tmp / 2.0;
  }

  c_tmp = q_ct_re_tmp * ct->f182.re - q_ct_im_tmp * ct->f182.im;
  b_c_tmp = q_ct_re_tmp * ct->f182.im + q_ct_im_tmp * ct->f182.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f393.re = c_tmp / 2.0;
    expl_temp.f393.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f393.re = 0.0;
    expl_temp.f393.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f393.re = c_tmp / 2.0;
    expl_temp.f393.im = b_c_tmp / 2.0;
  }

  c_tmp = p_ct_re_tmp * ct->f181.re - p_ct_im_tmp * ct->f181.im;
  b_c_tmp = p_ct_re_tmp * ct->f181.im + p_ct_im_tmp * ct->f181.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f392.re = c_tmp / 2.0;
    expl_temp.f392.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f392.re = 0.0;
    expl_temp.f392.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f392.re = c_tmp / 2.0;
    expl_temp.f392.im = b_c_tmp / 2.0;
  }

  c_tmp = r_ct_re_tmp * ct->f181.re - c_c_tmp * ct->f181.im;
  b_c_tmp = r_ct_re_tmp * ct->f181.im + c_c_tmp * ct->f181.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f391.re = c_tmp / 2.0;
    expl_temp.f391.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f391.re = 0.0;
    expl_temp.f391.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f391.re = c_tmp / 2.0;
    expl_temp.f391.im = b_c_tmp / 2.0;
  }

  c_tmp = ct_re_tmp * ct->f180.re - ct_im_tmp * ct->f180.im;
  b_c_tmp = ct_re_tmp * ct->f180.im + ct_im_tmp * ct->f180.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f390.re = c_tmp / 2.0;
    expl_temp.f390.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f390.re = 0.0;
    expl_temp.f390.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f390.re = c_tmp / 2.0;
    expl_temp.f390.im = b_c_tmp / 2.0;
  }

  c_tmp = c_ct_re_tmp * ct->f180.re - c_ct_im_tmp * ct->f180.im;
  b_c_tmp = c_ct_re_tmp * ct->f180.im + c_ct_im_tmp * ct->f180.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f389.re = c_tmp / 2.0;
    expl_temp.f389.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f389.re = 0.0;
    expl_temp.f389.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f389.re = c_tmp / 2.0;
    expl_temp.f389.im = b_c_tmp / 2.0;
  }

  c_tmp = b_ct_re_tmp * ct->f178.re - b_ct_im_tmp * ct->f178.im;
  b_c_tmp = b_ct_re_tmp * ct->f178.im + b_ct_im_tmp * ct->f178.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f388.re = c_tmp / 2.0;
    expl_temp.f388.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f388.re = 0.0;
    expl_temp.f388.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f388.re = c_tmp / 2.0;
    expl_temp.f388.im = b_c_tmp / 2.0;
  }

  c_tmp = g_ct_re_tmp * ct->f178.re - g_ct_im_tmp * ct->f178.im;
  b_c_tmp = g_ct_re_tmp * ct->f178.im + g_ct_im_tmp * ct->f178.re;
  if (b_c_tmp == 0.0) {
    expl_temp.f387.re = c_tmp / 2.0;
    expl_temp.f387.im = 0.0;
  } else if (c_tmp == 0.0) {
    expl_temp.f387.re = 0.0;
    expl_temp.f387.im = b_c_tmp / 2.0;
  } else {
    expl_temp.f387.re = c_tmp / 2.0;
    expl_temp.f387.im = b_c_tmp / 2.0;
  }

  b_c_tmp = ct->f415 * ct->f435;
  c_tmp = b_c_tmp * ct->f474 * ct->f498;
  c_c_tmp = c_tmp * ct->f177.re;
  c_tmp *= ct->f177.im;
  if (c_tmp == 0.0) {
    expl_temp.f386.re = c_c_tmp / 2.0;
    expl_temp.f386.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f386.re = 0.0;
    expl_temp.f386.im = c_tmp / 2.0;
  } else {
    expl_temp.f386.re = c_c_tmp / 2.0;
    expl_temp.f386.im = c_tmp / 2.0;
  }

  d_c_tmp = ct->f414 * ct->f435;
  c_tmp = d_c_tmp * ct->f474 * ct->f498;
  c_c_tmp = c_tmp * ct->f177.re;
  c_tmp *= ct->f177.im;
  if (c_tmp == 0.0) {
    expl_temp.f385.re = c_c_tmp / 2.0;
    expl_temp.f385.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f385.re = 0.0;
    expl_temp.f385.im = c_tmp / 2.0;
  } else {
    expl_temp.f385.re = c_c_tmp / 2.0;
    expl_temp.f385.im = c_tmp / 2.0;
  }

  ct_re_tmp = ct->f415 * ct->f434;
  c_tmp = ct_re_tmp * ct->f474 * ct->f498;
  c_c_tmp = c_tmp * ct->f177.re;
  c_tmp *= ct->f177.im;
  if (c_tmp == 0.0) {
    expl_temp.f384.re = c_c_tmp / 2.0;
    expl_temp.f384.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f384.re = 0.0;
    expl_temp.f384.im = c_tmp / 2.0;
  } else {
    expl_temp.f384.re = c_c_tmp / 2.0;
    expl_temp.f384.im = c_tmp / 2.0;
  }

  c_tmp = b_c_tmp * ct->f473 * ct->f498;
  c_c_tmp = c_tmp * ct->f177.re;
  c_tmp *= ct->f177.im;
  if (c_tmp == 0.0) {
    expl_temp.f383.re = c_c_tmp / 2.0;
    expl_temp.f383.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f383.re = 0.0;
    expl_temp.f383.im = c_tmp / 2.0;
  } else {
    expl_temp.f383.re = c_c_tmp / 2.0;
    expl_temp.f383.im = c_tmp / 2.0;
  }

  b_c_tmp = ct->f414 * ct->f434;
  c_tmp = b_c_tmp * ct->f474 * ct->f498;
  c_c_tmp = c_tmp * ct->f177.re;
  c_tmp *= ct->f177.im;
  if (c_tmp == 0.0) {
    expl_temp.f382.re = c_c_tmp / 2.0;
    expl_temp.f382.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f382.re = 0.0;
    expl_temp.f382.im = c_tmp / 2.0;
  } else {
    expl_temp.f382.re = c_c_tmp / 2.0;
    expl_temp.f382.im = c_tmp / 2.0;
  }

  c_tmp = d_c_tmp * ct->f473 * ct->f498;
  c_c_tmp = c_tmp * ct->f177.re;
  c_tmp *= ct->f177.im;
  if (c_tmp == 0.0) {
    expl_temp.f381.re = c_c_tmp / 2.0;
    expl_temp.f381.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f381.re = 0.0;
    expl_temp.f381.im = c_tmp / 2.0;
  } else {
    expl_temp.f381.re = c_c_tmp / 2.0;
    expl_temp.f381.im = c_tmp / 2.0;
  }

  c_tmp = ct_re_tmp * ct->f473 * ct->f498;
  c_c_tmp = c_tmp * ct->f177.re;
  c_tmp *= ct->f177.im;
  if (c_tmp == 0.0) {
    expl_temp.f380.re = c_c_tmp / 2.0;
    expl_temp.f380.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f380.re = 0.0;
    expl_temp.f380.im = c_tmp / 2.0;
  } else {
    expl_temp.f380.re = c_c_tmp / 2.0;
    expl_temp.f380.im = c_tmp / 2.0;
  }

  c_tmp = b_c_tmp * ct->f473 * ct->f498;
  c_c_tmp = c_tmp * ct->f177.re;
  c_tmp *= ct->f177.im;
  if (c_tmp == 0.0) {
    expl_temp.f379.re = c_c_tmp / 2.0;
    expl_temp.f379.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f379.re = 0.0;
    expl_temp.f379.im = c_tmp / 2.0;
  } else {
    expl_temp.f379.re = c_c_tmp / 2.0;
    expl_temp.f379.im = c_tmp / 2.0;
  }

  b_c_tmp = ct->f413 * ct->f432;
  c_tmp = b_c_tmp * ct->f472 * ct->f496;
  c_c_tmp = c_tmp * ct->f176.re;
  c_tmp *= ct->f176.im;
  if (c_tmp == 0.0) {
    expl_temp.f378.re = c_c_tmp / 2.0;
    expl_temp.f378.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f378.re = 0.0;
    expl_temp.f378.im = c_tmp / 2.0;
  } else {
    expl_temp.f378.re = c_c_tmp / 2.0;
    expl_temp.f378.im = c_tmp / 2.0;
  }

  d_c_tmp = ct->f404 * ct->f432;
  c_tmp = d_c_tmp * ct->f472 * ct->f496;
  c_c_tmp = c_tmp * ct->f176.re;
  c_tmp *= ct->f176.im;
  if (c_tmp == 0.0) {
    expl_temp.f377.re = c_c_tmp / 2.0;
    expl_temp.f377.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f377.re = 0.0;
    expl_temp.f377.im = c_tmp / 2.0;
  } else {
    expl_temp.f377.re = c_c_tmp / 2.0;
    expl_temp.f377.im = c_tmp / 2.0;
  }

  ct_re_tmp = ct->f413 * ct->f424;
  c_tmp = ct_re_tmp * ct->f472 * ct->f496;
  c_c_tmp = c_tmp * ct->f176.re;
  c_tmp *= ct->f176.im;
  if (c_tmp == 0.0) {
    expl_temp.f376.re = c_c_tmp / 2.0;
    expl_temp.f376.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f376.re = 0.0;
    expl_temp.f376.im = c_tmp / 2.0;
  } else {
    expl_temp.f376.re = c_c_tmp / 2.0;
    expl_temp.f376.im = c_tmp / 2.0;
  }

  c_tmp = b_c_tmp * ct->f463 * ct->f496;
  c_c_tmp = c_tmp * ct->f176.re;
  c_tmp *= ct->f176.im;
  if (c_tmp == 0.0) {
    expl_temp.f375.re = c_c_tmp / 2.0;
    expl_temp.f375.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f375.re = 0.0;
    expl_temp.f375.im = c_tmp / 2.0;
  } else {
    expl_temp.f375.re = c_c_tmp / 2.0;
    expl_temp.f375.im = c_tmp / 2.0;
  }

  b_c_tmp = ct->f412 * ct->f431;
  c_tmp = b_c_tmp * ct->f471 * ct->f493;
  c_c_tmp = c_tmp * ct->f175.re;
  c_tmp *= ct->f175.im;
  if (c_tmp == 0.0) {
    expl_temp.f374.re = c_c_tmp / 2.0;
    expl_temp.f374.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f374.re = 0.0;
    expl_temp.f374.im = c_tmp / 2.0;
  } else {
    expl_temp.f374.re = c_c_tmp / 2.0;
    expl_temp.f374.im = c_tmp / 2.0;
  }

  ct_im_tmp = ct->f404 * ct->f424;
  c_tmp = ct_im_tmp * ct->f472 * ct->f496;
  c_c_tmp = c_tmp * ct->f176.re;
  c_tmp *= ct->f176.im;
  if (c_tmp == 0.0) {
    expl_temp.f373.re = c_c_tmp / 2.0;
    expl_temp.f373.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f373.re = 0.0;
    expl_temp.f373.im = c_tmp / 2.0;
  } else {
    expl_temp.f373.re = c_c_tmp / 2.0;
    expl_temp.f373.im = c_tmp / 2.0;
  }

  c_tmp = d_c_tmp * ct->f463 * ct->f496;
  c_c_tmp = c_tmp * ct->f176.re;
  c_tmp *= ct->f176.im;
  if (c_tmp == 0.0) {
    expl_temp.f372.re = c_c_tmp / 2.0;
    expl_temp.f372.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f372.re = 0.0;
    expl_temp.f372.im = c_tmp / 2.0;
  } else {
    expl_temp.f372.re = c_c_tmp / 2.0;
    expl_temp.f372.im = c_tmp / 2.0;
  }

  d_c_tmp = ct->f403 * ct->f431;
  c_tmp = d_c_tmp * ct->f471 * ct->f493;
  c_c_tmp = c_tmp * ct->f175.re;
  c_tmp *= ct->f175.im;
  if (c_tmp == 0.0) {
    expl_temp.f371.re = c_c_tmp / 2.0;
    expl_temp.f371.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f371.re = 0.0;
    expl_temp.f371.im = c_tmp / 2.0;
  } else {
    expl_temp.f371.re = c_c_tmp / 2.0;
    expl_temp.f371.im = c_tmp / 2.0;
  }

  c_tmp = ct_re_tmp * ct->f463 * ct->f496;
  c_c_tmp = c_tmp * ct->f176.re;
  c_tmp *= ct->f176.im;
  if (c_tmp == 0.0) {
    expl_temp.f370.re = c_c_tmp / 2.0;
    expl_temp.f370.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f370.re = 0.0;
    expl_temp.f370.im = c_tmp / 2.0;
  } else {
    expl_temp.f370.re = c_c_tmp / 2.0;
    expl_temp.f370.im = c_tmp / 2.0;
  }

  ct_re_tmp = ct->f412 * ct->f423;
  c_tmp = ct_re_tmp * ct->f471 * ct->f493;
  c_c_tmp = c_tmp * ct->f175.re;
  c_tmp *= ct->f175.im;
  if (c_tmp == 0.0) {
    expl_temp.f369.re = c_c_tmp / 2.0;
    expl_temp.f369.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f369.re = 0.0;
    expl_temp.f369.im = c_tmp / 2.0;
  } else {
    expl_temp.f369.re = c_c_tmp / 2.0;
    expl_temp.f369.im = c_tmp / 2.0;
  }

  c_tmp = b_c_tmp * ct->f462 * ct->f493;
  c_c_tmp = c_tmp * ct->f175.re;
  c_tmp *= ct->f175.im;
  if (c_tmp == 0.0) {
    expl_temp.f368.re = c_c_tmp / 2.0;
    expl_temp.f368.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f368.re = 0.0;
    expl_temp.f368.im = c_tmp / 2.0;
  } else {
    expl_temp.f368.re = c_c_tmp / 2.0;
    expl_temp.f368.im = c_tmp / 2.0;
  }

  b_c_tmp = ct->f410 * ct->f430;
  c_tmp = b_c_tmp * ct->f470 * ct->f491;
  c_c_tmp = c_tmp * ct->f174.re;
  c_tmp *= ct->f174.im;
  if (c_tmp == 0.0) {
    expl_temp.f367.re = c_c_tmp / 2.0;
    expl_temp.f367.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f367.re = 0.0;
    expl_temp.f367.im = c_tmp / 2.0;
  } else {
    expl_temp.f367.re = c_c_tmp / 2.0;
    expl_temp.f367.im = c_tmp / 2.0;
  }

  c_tmp = ct_im_tmp * ct->f463 * ct->f496;
  c_c_tmp = c_tmp * ct->f176.re;
  c_tmp *= ct->f176.im;
  if (c_tmp == 0.0) {
    expl_temp.f366.re = c_c_tmp / 2.0;
    expl_temp.f366.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f366.re = 0.0;
    expl_temp.f366.im = c_tmp / 2.0;
  } else {
    expl_temp.f366.re = c_c_tmp / 2.0;
    expl_temp.f366.im = c_tmp / 2.0;
  }

  ct_im_tmp = ct->f403 * ct->f423;
  c_tmp = ct_im_tmp * ct->f471 * ct->f493;
  c_c_tmp = c_tmp * ct->f175.re;
  c_tmp *= ct->f175.im;
  if (c_tmp == 0.0) {
    expl_temp.f365.re = c_c_tmp / 2.0;
    expl_temp.f365.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f365.re = 0.0;
    expl_temp.f365.im = c_tmp / 2.0;
  } else {
    expl_temp.f365.re = c_c_tmp / 2.0;
    expl_temp.f365.im = c_tmp / 2.0;
  }

  c_tmp = d_c_tmp * ct->f462 * ct->f493;
  c_c_tmp = c_tmp * ct->f175.re;
  c_tmp *= ct->f175.im;
  if (c_tmp == 0.0) {
    expl_temp.f364.re = c_c_tmp / 2.0;
    expl_temp.f364.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f364.re = 0.0;
    expl_temp.f364.im = c_tmp / 2.0;
  } else {
    expl_temp.f364.re = c_c_tmp / 2.0;
    expl_temp.f364.im = c_tmp / 2.0;
  }

  d_c_tmp = ct->f402 * ct->f430;
  c_tmp = d_c_tmp * ct->f470 * ct->f491;
  c_c_tmp = c_tmp * ct->f174.re;
  c_tmp *= ct->f174.im;
  if (c_tmp == 0.0) {
    expl_temp.f363.re = c_c_tmp / 2.0;
    expl_temp.f363.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f363.re = 0.0;
    expl_temp.f363.im = c_tmp / 2.0;
  } else {
    expl_temp.f363.re = c_c_tmp / 2.0;
    expl_temp.f363.im = c_tmp / 2.0;
  }

  c_tmp = ct_re_tmp * ct->f462 * ct->f493;
  c_c_tmp = c_tmp * ct->f175.re;
  c_tmp *= ct->f175.im;
  if (c_tmp == 0.0) {
    expl_temp.f362.re = c_c_tmp / 2.0;
    expl_temp.f362.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f362.re = 0.0;
    expl_temp.f362.im = c_tmp / 2.0;
  } else {
    expl_temp.f362.re = c_c_tmp / 2.0;
    expl_temp.f362.im = c_tmp / 2.0;
  }

  ct_re_tmp = ct->f410 * ct->f421;
  c_tmp = ct_re_tmp * ct->f470 * ct->f491;
  c_c_tmp = c_tmp * ct->f174.re;
  c_tmp *= ct->f174.im;
  if (c_tmp == 0.0) {
    expl_temp.f361.re = c_c_tmp / 2.0;
    expl_temp.f361.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f361.re = 0.0;
    expl_temp.f361.im = c_tmp / 2.0;
  } else {
    expl_temp.f361.re = c_c_tmp / 2.0;
    expl_temp.f361.im = c_tmp / 2.0;
  }

  c_tmp = b_c_tmp * ct->f461 * ct->f491;
  c_c_tmp = c_tmp * ct->f174.re;
  c_tmp *= ct->f174.im;
  if (c_tmp == 0.0) {
    expl_temp.f360.re = c_c_tmp / 2.0;
    expl_temp.f360.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f360.re = 0.0;
    expl_temp.f360.im = c_tmp / 2.0;
  } else {
    expl_temp.f360.re = c_c_tmp / 2.0;
    expl_temp.f360.im = c_tmp / 2.0;
  }

  b_c_tmp = ct->f409 * ct->f429;
  c_tmp = b_c_tmp * ct->f469 * ct->f489;
  c_c_tmp = c_tmp * ct->f173.re;
  c_tmp *= ct->f173.im;
  if (c_tmp == 0.0) {
    expl_temp.f359.re = c_c_tmp / 2.0;
    expl_temp.f359.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f359.re = 0.0;
    expl_temp.f359.im = c_tmp / 2.0;
  } else {
    expl_temp.f359.re = c_c_tmp / 2.0;
    expl_temp.f359.im = c_tmp / 2.0;
  }

  c_tmp = ct_im_tmp * ct->f462 * ct->f493;
  c_c_tmp = c_tmp * ct->f175.re;
  c_tmp *= ct->f175.im;
  if (c_tmp == 0.0) {
    expl_temp.f358.re = c_c_tmp / 2.0;
    expl_temp.f358.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f358.re = 0.0;
    expl_temp.f358.im = c_tmp / 2.0;
  } else {
    expl_temp.f358.re = c_c_tmp / 2.0;
    expl_temp.f358.im = c_tmp / 2.0;
  }

  ct_im_tmp = ct->f402 * ct->f421;
  c_tmp = ct_im_tmp * ct->f470 * ct->f491;
  c_c_tmp = c_tmp * ct->f174.re;
  c_tmp *= ct->f174.im;
  if (c_tmp == 0.0) {
    expl_temp.f357.re = c_c_tmp / 2.0;
    expl_temp.f357.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f357.re = 0.0;
    expl_temp.f357.im = c_tmp / 2.0;
  } else {
    expl_temp.f357.re = c_c_tmp / 2.0;
    expl_temp.f357.im = c_tmp / 2.0;
  }

  c_tmp = d_c_tmp * ct->f461 * ct->f491;
  c_c_tmp = c_tmp * ct->f174.re;
  c_tmp *= ct->f174.im;
  if (c_tmp == 0.0) {
    expl_temp.f356.re = c_c_tmp / 2.0;
    expl_temp.f356.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f356.re = 0.0;
    expl_temp.f356.im = c_tmp / 2.0;
  } else {
    expl_temp.f356.re = c_c_tmp / 2.0;
    expl_temp.f356.im = c_tmp / 2.0;
  }

  d_c_tmp = ct->f401 * ct->f429;
  c_tmp = d_c_tmp * ct->f469 * ct->f489;
  c_c_tmp = c_tmp * ct->f173.re;
  c_tmp *= ct->f173.im;
  if (c_tmp == 0.0) {
    expl_temp.f355.re = c_c_tmp / 2.0;
    expl_temp.f355.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f355.re = 0.0;
    expl_temp.f355.im = c_tmp / 2.0;
  } else {
    expl_temp.f355.re = c_c_tmp / 2.0;
    expl_temp.f355.im = c_tmp / 2.0;
  }

  c_tmp = ct_re_tmp * ct->f461 * ct->f491;
  c_c_tmp = c_tmp * ct->f174.re;
  c_tmp *= ct->f174.im;
  if (c_tmp == 0.0) {
    expl_temp.f354.re = c_c_tmp / 2.0;
    expl_temp.f354.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f354.re = 0.0;
    expl_temp.f354.im = c_tmp / 2.0;
  } else {
    expl_temp.f354.re = c_c_tmp / 2.0;
    expl_temp.f354.im = c_tmp / 2.0;
  }

  ct_re_tmp = ct->f409 * ct->f420;
  c_tmp = ct_re_tmp * ct->f469 * ct->f489;
  c_c_tmp = c_tmp * ct->f173.re;
  c_tmp *= ct->f173.im;
  if (c_tmp == 0.0) {
    expl_temp.f353.re = c_c_tmp / 2.0;
    expl_temp.f353.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f353.re = 0.0;
    expl_temp.f353.im = c_tmp / 2.0;
  } else {
    expl_temp.f353.re = c_c_tmp / 2.0;
    expl_temp.f353.im = c_tmp / 2.0;
  }

  c_tmp = b_c_tmp * ct->f460 * ct->f489;
  c_c_tmp = c_tmp * ct->f173.re;
  c_tmp *= ct->f173.im;
  if (c_tmp == 0.0) {
    expl_temp.f352.re = c_c_tmp / 2.0;
    expl_temp.f352.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f352.re = 0.0;
    expl_temp.f352.im = c_tmp / 2.0;
  } else {
    expl_temp.f352.re = c_c_tmp / 2.0;
    expl_temp.f352.im = c_tmp / 2.0;
  }

  b_c_tmp = ct->f408 * ct->f428;
  c_tmp = b_c_tmp * ct->f468 * ct->f487;
  c_c_tmp = c_tmp * ct->f172.re;
  c_tmp *= ct->f172.im;
  if (c_tmp == 0.0) {
    expl_temp.f351.re = c_c_tmp / 2.0;
    expl_temp.f351.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f351.re = 0.0;
    expl_temp.f351.im = c_tmp / 2.0;
  } else {
    expl_temp.f351.re = c_c_tmp / 2.0;
    expl_temp.f351.im = c_tmp / 2.0;
  }

  c_tmp = ct_im_tmp * ct->f461 * ct->f491;
  c_c_tmp = c_tmp * ct->f174.re;
  c_tmp *= ct->f174.im;
  if (c_tmp == 0.0) {
    expl_temp.f350.re = c_c_tmp / 2.0;
    expl_temp.f350.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f350.re = 0.0;
    expl_temp.f350.im = c_tmp / 2.0;
  } else {
    expl_temp.f350.re = c_c_tmp / 2.0;
    expl_temp.f350.im = c_tmp / 2.0;
  }

  ct_im_tmp = ct->f401 * ct->f420;
  c_tmp = ct_im_tmp * ct->f469 * ct->f489;
  c_c_tmp = c_tmp * ct->f173.re;
  c_tmp *= ct->f173.im;
  if (c_tmp == 0.0) {
    expl_temp.f349.re = c_c_tmp / 2.0;
    expl_temp.f349.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f349.re = 0.0;
    expl_temp.f349.im = c_tmp / 2.0;
  } else {
    expl_temp.f349.re = c_c_tmp / 2.0;
    expl_temp.f349.im = c_tmp / 2.0;
  }

  c_tmp = d_c_tmp * ct->f460 * ct->f489;
  c_c_tmp = c_tmp * ct->f173.re;
  c_tmp *= ct->f173.im;
  if (c_tmp == 0.0) {
    expl_temp.f348.re = c_c_tmp / 2.0;
    expl_temp.f348.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f348.re = 0.0;
    expl_temp.f348.im = c_tmp / 2.0;
  } else {
    expl_temp.f348.re = c_c_tmp / 2.0;
    expl_temp.f348.im = c_tmp / 2.0;
  }

  d_c_tmp = ct->f399 * ct->f428;
  c_tmp = d_c_tmp * ct->f468 * ct->f487;
  c_c_tmp = c_tmp * ct->f172.re;
  c_tmp *= ct->f172.im;
  if (c_tmp == 0.0) {
    expl_temp.f347.re = c_c_tmp / 2.0;
    expl_temp.f347.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f347.re = 0.0;
    expl_temp.f347.im = c_tmp / 2.0;
  } else {
    expl_temp.f347.re = c_c_tmp / 2.0;
    expl_temp.f347.im = c_tmp / 2.0;
  }

  c_tmp = ct_re_tmp * ct->f460 * ct->f489;
  c_c_tmp = c_tmp * ct->f173.re;
  c_tmp *= ct->f173.im;
  if (c_tmp == 0.0) {
    expl_temp.f346.re = c_c_tmp / 2.0;
    expl_temp.f346.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f346.re = 0.0;
    expl_temp.f346.im = c_tmp / 2.0;
  } else {
    expl_temp.f346.re = c_c_tmp / 2.0;
    expl_temp.f346.im = c_tmp / 2.0;
  }

  ct_re_tmp = ct->f408 * ct->f419;
  c_tmp = ct_re_tmp * ct->f468 * ct->f487;
  c_c_tmp = c_tmp * ct->f172.re;
  c_tmp *= ct->f172.im;
  if (c_tmp == 0.0) {
    expl_temp.f345.re = c_c_tmp / 2.0;
    expl_temp.f345.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f345.re = 0.0;
    expl_temp.f345.im = c_tmp / 2.0;
  } else {
    expl_temp.f345.re = c_c_tmp / 2.0;
    expl_temp.f345.im = c_tmp / 2.0;
  }

  c_tmp = b_c_tmp * ct->f459 * ct->f487;
  c_c_tmp = c_tmp * ct->f172.re;
  c_tmp *= ct->f172.im;
  if (c_tmp == 0.0) {
    expl_temp.f344.re = c_c_tmp / 2.0;
    expl_temp.f344.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f344.re = 0.0;
    expl_temp.f344.im = c_tmp / 2.0;
  } else {
    expl_temp.f344.re = c_c_tmp / 2.0;
    expl_temp.f344.im = c_tmp / 2.0;
  }

  b_c_tmp = ct->f407 * ct->f427;
  c_tmp = b_c_tmp * ct->f467 * ct->f485;
  c_c_tmp = c_tmp * ct->f171.re;
  c_tmp *= ct->f171.im;
  if (c_tmp == 0.0) {
    expl_temp.f343.re = c_c_tmp / 2.0;
    expl_temp.f343.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f343.re = 0.0;
    expl_temp.f343.im = c_tmp / 2.0;
  } else {
    expl_temp.f343.re = c_c_tmp / 2.0;
    expl_temp.f343.im = c_tmp / 2.0;
  }

  c_tmp = ct_im_tmp * ct->f460 * ct->f489;
  c_c_tmp = c_tmp * ct->f173.re;
  c_tmp *= ct->f173.im;
  if (c_tmp == 0.0) {
    expl_temp.f342.re = c_c_tmp / 2.0;
    expl_temp.f342.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f342.re = 0.0;
    expl_temp.f342.im = c_tmp / 2.0;
  } else {
    expl_temp.f342.re = c_c_tmp / 2.0;
    expl_temp.f342.im = c_tmp / 2.0;
  }

  ct_im_tmp = ct->f399 * ct->f419;
  c_tmp = ct_im_tmp * ct->f468 * ct->f487;
  c_c_tmp = c_tmp * ct->f172.re;
  c_tmp *= ct->f172.im;
  if (c_tmp == 0.0) {
    expl_temp.f341.re = c_c_tmp / 2.0;
    expl_temp.f341.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f341.re = 0.0;
    expl_temp.f341.im = c_tmp / 2.0;
  } else {
    expl_temp.f341.re = c_c_tmp / 2.0;
    expl_temp.f341.im = c_tmp / 2.0;
  }

  c_tmp = d_c_tmp * ct->f459 * ct->f487;
  c_c_tmp = c_tmp * ct->f172.re;
  c_tmp *= ct->f172.im;
  if (c_tmp == 0.0) {
    expl_temp.f340.re = c_c_tmp / 2.0;
    expl_temp.f340.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f340.re = 0.0;
    expl_temp.f340.im = c_tmp / 2.0;
  } else {
    expl_temp.f340.re = c_c_tmp / 2.0;
    expl_temp.f340.im = c_tmp / 2.0;
  }

  d_c_tmp = ct->f398 * ct->f427;
  c_tmp = d_c_tmp * ct->f467 * ct->f485;
  c_c_tmp = c_tmp * ct->f171.re;
  c_tmp *= ct->f171.im;
  if (c_tmp == 0.0) {
    expl_temp.f339.re = c_c_tmp / 2.0;
    expl_temp.f339.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f339.re = 0.0;
    expl_temp.f339.im = c_tmp / 2.0;
  } else {
    expl_temp.f339.re = c_c_tmp / 2.0;
    expl_temp.f339.im = c_tmp / 2.0;
  }

  c_tmp = ct_re_tmp * ct->f459 * ct->f487;
  c_c_tmp = c_tmp * ct->f172.re;
  c_tmp *= ct->f172.im;
  if (c_tmp == 0.0) {
    expl_temp.f338.re = c_c_tmp / 2.0;
    expl_temp.f338.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f338.re = 0.0;
    expl_temp.f338.im = c_tmp / 2.0;
  } else {
    expl_temp.f338.re = c_c_tmp / 2.0;
    expl_temp.f338.im = c_tmp / 2.0;
  }

  ct_re_tmp = ct->f407 * ct->f418;
  c_tmp = ct_re_tmp * ct->f467 * ct->f485;
  c_c_tmp = c_tmp * ct->f171.re;
  c_tmp *= ct->f171.im;
  if (c_tmp == 0.0) {
    expl_temp.f337.re = c_c_tmp / 2.0;
    expl_temp.f337.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f337.re = 0.0;
    expl_temp.f337.im = c_tmp / 2.0;
  } else {
    expl_temp.f337.re = c_c_tmp / 2.0;
    expl_temp.f337.im = c_tmp / 2.0;
  }

  c_tmp = b_c_tmp * ct->f458 * ct->f485;
  c_c_tmp = c_tmp * ct->f171.re;
  c_tmp *= ct->f171.im;
  if (c_tmp == 0.0) {
    expl_temp.f336.re = c_c_tmp / 2.0;
    expl_temp.f336.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f336.re = 0.0;
    expl_temp.f336.im = c_tmp / 2.0;
  } else {
    expl_temp.f336.re = c_c_tmp / 2.0;
    expl_temp.f336.im = c_tmp / 2.0;
  }

  b_c_tmp = ct->f406 * ct->f426;
  c_tmp = b_c_tmp * ct->f465 * ct->f481;
  c_c_tmp = c_tmp * ct->f170.re;
  c_tmp *= ct->f170.im;
  if (c_tmp == 0.0) {
    expl_temp.f335.re = c_c_tmp / 2.0;
    expl_temp.f335.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f335.re = 0.0;
    expl_temp.f335.im = c_tmp / 2.0;
  } else {
    expl_temp.f335.re = c_c_tmp / 2.0;
    expl_temp.f335.im = c_tmp / 2.0;
  }

  c_tmp = ct_im_tmp * ct->f459 * ct->f487;
  c_c_tmp = c_tmp * ct->f172.re;
  c_tmp *= ct->f172.im;
  if (c_tmp == 0.0) {
    expl_temp.f334.re = c_c_tmp / 2.0;
    expl_temp.f334.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f334.re = 0.0;
    expl_temp.f334.im = c_tmp / 2.0;
  } else {
    expl_temp.f334.re = c_c_tmp / 2.0;
    expl_temp.f334.im = c_tmp / 2.0;
  }

  ct_im_tmp = ct->f398 * ct->f418;
  c_tmp = ct_im_tmp * ct->f467 * ct->f485;
  c_c_tmp = c_tmp * ct->f171.re;
  c_tmp *= ct->f171.im;
  if (c_tmp == 0.0) {
    expl_temp.f333.re = c_c_tmp / 2.0;
    expl_temp.f333.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f333.re = 0.0;
    expl_temp.f333.im = c_tmp / 2.0;
  } else {
    expl_temp.f333.re = c_c_tmp / 2.0;
    expl_temp.f333.im = c_tmp / 2.0;
  }

  c_tmp = d_c_tmp * ct->f458 * ct->f485;
  c_c_tmp = c_tmp * ct->f171.re;
  c_tmp *= ct->f171.im;
  if (c_tmp == 0.0) {
    expl_temp.f332.re = c_c_tmp / 2.0;
    expl_temp.f332.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f332.re = 0.0;
    expl_temp.f332.im = c_tmp / 2.0;
  } else {
    expl_temp.f332.re = c_c_tmp / 2.0;
    expl_temp.f332.im = c_tmp / 2.0;
  }

  d_c_tmp = ct->f397 * ct->f426;
  c_tmp = d_c_tmp * ct->f465 * ct->f481;
  c_c_tmp = c_tmp * ct->f170.re;
  c_tmp *= ct->f170.im;
  if (c_tmp == 0.0) {
    expl_temp.f331.re = c_c_tmp / 2.0;
    expl_temp.f331.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f331.re = 0.0;
    expl_temp.f331.im = c_tmp / 2.0;
  } else {
    expl_temp.f331.re = c_c_tmp / 2.0;
    expl_temp.f331.im = c_tmp / 2.0;
  }

  c_tmp = ct_re_tmp * ct->f458 * ct->f485;
  c_c_tmp = c_tmp * ct->f171.re;
  c_tmp *= ct->f171.im;
  if (c_tmp == 0.0) {
    expl_temp.f330.re = c_c_tmp / 2.0;
    expl_temp.f330.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f330.re = 0.0;
    expl_temp.f330.im = c_tmp / 2.0;
  } else {
    expl_temp.f330.re = c_c_tmp / 2.0;
    expl_temp.f330.im = c_tmp / 2.0;
  }

  ct_re_tmp = ct->f406 * ct->f417;
  c_tmp = ct_re_tmp * ct->f465 * ct->f481;
  c_c_tmp = c_tmp * ct->f170.re;
  c_tmp *= ct->f170.im;
  if (c_tmp == 0.0) {
    expl_temp.f329.re = c_c_tmp / 2.0;
    expl_temp.f329.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f329.re = 0.0;
    expl_temp.f329.im = c_tmp / 2.0;
  } else {
    expl_temp.f329.re = c_c_tmp / 2.0;
    expl_temp.f329.im = c_tmp / 2.0;
  }

  c_tmp = b_c_tmp * ct->f457 * ct->f481;
  c_c_tmp = c_tmp * ct->f170.re;
  c_tmp *= ct->f170.im;
  if (c_tmp == 0.0) {
    expl_temp.f328.re = c_c_tmp / 2.0;
    expl_temp.f328.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f328.re = 0.0;
    expl_temp.f328.im = c_tmp / 2.0;
  } else {
    expl_temp.f328.re = c_c_tmp / 2.0;
    expl_temp.f328.im = c_tmp / 2.0;
  }

  expl_temp.f327 = ct->f222;
  expl_temp.f326 = ct->f221;
  b_c_tmp = ct->f405 * ct->f425;
  c_tmp = b_c_tmp * ct->f464 * ct->f479;
  c_c_tmp = c_tmp * ct->f169.re;
  c_tmp *= ct->f169.im;
  if (c_tmp == 0.0) {
    expl_temp.f325.re = c_c_tmp / 2.0;
    expl_temp.f325.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f325.re = 0.0;
    expl_temp.f325.im = c_tmp / 2.0;
  } else {
    expl_temp.f325.re = c_c_tmp / 2.0;
    expl_temp.f325.im = c_tmp / 2.0;
  }

  c_tmp = ct_im_tmp * ct->f458 * ct->f485;
  c_c_tmp = c_tmp * ct->f171.re;
  c_tmp *= ct->f171.im;
  if (c_tmp == 0.0) {
    expl_temp.f324.re = c_c_tmp / 2.0;
    expl_temp.f324.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f324.re = 0.0;
    expl_temp.f324.im = c_tmp / 2.0;
  } else {
    expl_temp.f324.re = c_c_tmp / 2.0;
    expl_temp.f324.im = c_tmp / 2.0;
  }

  ct_im_tmp = ct->f397 * ct->f417;
  c_tmp = ct_im_tmp * ct->f465 * ct->f481;
  c_c_tmp = c_tmp * ct->f170.re;
  c_tmp *= ct->f170.im;
  if (c_tmp == 0.0) {
    expl_temp.f323.re = c_c_tmp / 2.0;
    expl_temp.f323.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f323.re = 0.0;
    expl_temp.f323.im = c_tmp / 2.0;
  } else {
    expl_temp.f323.re = c_c_tmp / 2.0;
    expl_temp.f323.im = c_tmp / 2.0;
  }

  c_tmp = d_c_tmp * ct->f457 * ct->f481;
  c_c_tmp = c_tmp * ct->f170.re;
  c_tmp *= ct->f170.im;
  if (c_tmp == 0.0) {
    expl_temp.f322.re = c_c_tmp / 2.0;
    expl_temp.f322.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f322.re = 0.0;
    expl_temp.f322.im = c_tmp / 2.0;
  } else {
    expl_temp.f322.re = c_c_tmp / 2.0;
    expl_temp.f322.im = c_tmp / 2.0;
  }

  d_c_tmp = ct->f396 * ct->f425;
  c_tmp = d_c_tmp * ct->f464 * ct->f479;
  c_c_tmp = c_tmp * ct->f169.re;
  c_tmp *= ct->f169.im;
  if (c_tmp == 0.0) {
    expl_temp.f321.re = c_c_tmp / 2.0;
    expl_temp.f321.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f321.re = 0.0;
    expl_temp.f321.im = c_tmp / 2.0;
  } else {
    expl_temp.f321.re = c_c_tmp / 2.0;
    expl_temp.f321.im = c_tmp / 2.0;
  }

  c_tmp = ct_re_tmp * ct->f457 * ct->f481;
  c_c_tmp = c_tmp * ct->f170.re;
  c_tmp *= ct->f170.im;
  if (c_tmp == 0.0) {
    expl_temp.f320.re = c_c_tmp / 2.0;
    expl_temp.f320.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f320.re = 0.0;
    expl_temp.f320.im = c_tmp / 2.0;
  } else {
    expl_temp.f320.re = c_c_tmp / 2.0;
    expl_temp.f320.im = c_tmp / 2.0;
  }

  ct_re_tmp = ct->f405 * ct->f416;
  c_tmp = ct_re_tmp * ct->f464 * ct->f479;
  c_c_tmp = c_tmp * ct->f169.re;
  c_tmp *= ct->f169.im;
  if (c_tmp == 0.0) {
    expl_temp.f319.re = c_c_tmp / 2.0;
    expl_temp.f319.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f319.re = 0.0;
    expl_temp.f319.im = c_tmp / 2.0;
  } else {
    expl_temp.f319.re = c_c_tmp / 2.0;
    expl_temp.f319.im = c_tmp / 2.0;
  }

  c_tmp = b_c_tmp * ct->f456 * ct->f479;
  c_c_tmp = c_tmp * ct->f169.re;
  c_tmp *= ct->f169.im;
  if (c_tmp == 0.0) {
    expl_temp.f318.re = c_c_tmp / 2.0;
    expl_temp.f318.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f318.re = 0.0;
    expl_temp.f318.im = c_tmp / 2.0;
  } else {
    expl_temp.f318.re = c_c_tmp / 2.0;
    expl_temp.f318.im = c_tmp / 2.0;
  }

  c_tmp = ct_im_tmp * ct->f457 * ct->f481;
  c_c_tmp = c_tmp * ct->f170.re;
  c_tmp *= ct->f170.im;
  if (c_tmp == 0.0) {
    expl_temp.f317.re = c_c_tmp / 2.0;
    expl_temp.f317.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f317.re = 0.0;
    expl_temp.f317.im = c_tmp / 2.0;
  } else {
    expl_temp.f317.re = c_c_tmp / 2.0;
    expl_temp.f317.im = c_tmp / 2.0;
  }

  b_c_tmp = ct->f396 * ct->f416;
  c_tmp = b_c_tmp * ct->f464 * ct->f479;
  c_c_tmp = c_tmp * ct->f169.re;
  c_tmp *= ct->f169.im;
  if (c_tmp == 0.0) {
    expl_temp.f316.re = c_c_tmp / 2.0;
    expl_temp.f316.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f316.re = 0.0;
    expl_temp.f316.im = c_tmp / 2.0;
  } else {
    expl_temp.f316.re = c_c_tmp / 2.0;
    expl_temp.f316.im = c_tmp / 2.0;
  }

  c_tmp = d_c_tmp * ct->f456 * ct->f479;
  c_c_tmp = c_tmp * ct->f169.re;
  c_tmp *= ct->f169.im;
  if (c_tmp == 0.0) {
    expl_temp.f315.re = c_c_tmp / 2.0;
    expl_temp.f315.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f315.re = 0.0;
    expl_temp.f315.im = c_tmp / 2.0;
  } else {
    expl_temp.f315.re = c_c_tmp / 2.0;
    expl_temp.f315.im = c_tmp / 2.0;
  }

  c_tmp = ct_re_tmp * ct->f456 * ct->f479;
  c_c_tmp = c_tmp * ct->f169.re;
  c_tmp *= ct->f169.im;
  if (c_tmp == 0.0) {
    expl_temp.f314.re = c_c_tmp / 2.0;
    expl_temp.f314.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f314.re = 0.0;
    expl_temp.f314.im = c_tmp / 2.0;
  } else {
    expl_temp.f314.re = c_c_tmp / 2.0;
    expl_temp.f314.im = c_tmp / 2.0;
  }

  c_tmp = b_c_tmp * ct->f456 * ct->f479;
  c_c_tmp = c_tmp * ct->f169.re;
  c_tmp *= ct->f169.im;
  if (c_tmp == 0.0) {
    expl_temp.f313.re = c_c_tmp / 2.0;
    expl_temp.f313.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f313.re = 0.0;
    expl_temp.f313.im = c_tmp / 2.0;
  } else {
    expl_temp.f313.re = c_c_tmp / 2.0;
    expl_temp.f313.im = c_tmp / 2.0;
  }

  c_tmp = ct->f454 * ct->f474 * ct->f498;
  c_c_tmp = c_tmp * ct->f177.re;
  c_tmp *= ct->f177.im;
  if (c_tmp == 0.0) {
    expl_temp.f312.re = c_c_tmp / 2.0;
    expl_temp.f312.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f312.re = 0.0;
    expl_temp.f312.im = c_tmp / 2.0;
  } else {
    expl_temp.f312.re = c_c_tmp / 2.0;
    expl_temp.f312.im = c_tmp / 2.0;
  }

  c_tmp = ct->f453 * ct->f474 * ct->f498;
  c_c_tmp = c_tmp * ct->f177.re;
  c_tmp *= ct->f177.im;
  if (c_tmp == 0.0) {
    expl_temp.f311.re = c_c_tmp / 2.0;
    expl_temp.f311.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f311.re = 0.0;
    expl_temp.f311.im = c_tmp / 2.0;
  } else {
    expl_temp.f311.re = c_c_tmp / 2.0;
    expl_temp.f311.im = c_tmp / 2.0;
  }

  c_tmp = ct->f454 * ct->f473 * ct->f498;
  c_c_tmp = c_tmp * ct->f177.re;
  c_tmp *= ct->f177.im;
  if (c_tmp == 0.0) {
    expl_temp.f310.re = c_c_tmp / 2.0;
    expl_temp.f310.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f310.re = 0.0;
    expl_temp.f310.im = c_tmp / 2.0;
  } else {
    expl_temp.f310.re = c_c_tmp / 2.0;
    expl_temp.f310.im = c_tmp / 2.0;
  }

  c_tmp = ct->f453 * ct->f473 * ct->f498;
  c_c_tmp = c_tmp * ct->f177.re;
  c_tmp *= ct->f177.im;
  if (c_tmp == 0.0) {
    expl_temp.f309.re = c_c_tmp / 2.0;
    expl_temp.f309.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f309.re = 0.0;
    expl_temp.f309.im = c_tmp / 2.0;
  } else {
    expl_temp.f309.re = c_c_tmp / 2.0;
    expl_temp.f309.im = c_tmp / 2.0;
  }

  c_tmp = ct->f452 * ct->f472 * ct->f496;
  c_c_tmp = c_tmp * ct->f176.re;
  c_tmp *= ct->f176.im;
  if (c_tmp == 0.0) {
    expl_temp.f308.re = c_c_tmp / 2.0;
    expl_temp.f308.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f308.re = 0.0;
    expl_temp.f308.im = c_tmp / 2.0;
  } else {
    expl_temp.f308.re = c_c_tmp / 2.0;
    expl_temp.f308.im = c_tmp / 2.0;
  }

  c_tmp = ct->f443 * ct->f472 * ct->f496;
  c_c_tmp = c_tmp * ct->f176.re;
  c_tmp *= ct->f176.im;
  if (c_tmp == 0.0) {
    expl_temp.f307.re = c_c_tmp / 2.0;
    expl_temp.f307.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f307.re = 0.0;
    expl_temp.f307.im = c_tmp / 2.0;
  } else {
    expl_temp.f307.re = c_c_tmp / 2.0;
    expl_temp.f307.im = c_tmp / 2.0;
  }

  c_tmp = ct->f452 * ct->f463 * ct->f496;
  c_c_tmp = c_tmp * ct->f176.re;
  c_tmp *= ct->f176.im;
  if (c_tmp == 0.0) {
    expl_temp.f306.re = c_c_tmp / 2.0;
    expl_temp.f306.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f306.re = 0.0;
    expl_temp.f306.im = c_tmp / 2.0;
  } else {
    expl_temp.f306.re = c_c_tmp / 2.0;
    expl_temp.f306.im = c_tmp / 2.0;
  }

  c_tmp = ct->f451 * ct->f471 * ct->f493;
  c_c_tmp = c_tmp * ct->f175.re;
  c_tmp *= ct->f175.im;
  if (c_tmp == 0.0) {
    expl_temp.f305.re = c_c_tmp / 2.0;
    expl_temp.f305.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f305.re = 0.0;
    expl_temp.f305.im = c_tmp / 2.0;
  } else {
    expl_temp.f305.re = c_c_tmp / 2.0;
    expl_temp.f305.im = c_tmp / 2.0;
  }

  c_tmp = ct->f443 * ct->f463 * ct->f496;
  c_c_tmp = c_tmp * ct->f176.re;
  c_tmp *= ct->f176.im;
  if (c_tmp == 0.0) {
    expl_temp.f304.re = c_c_tmp / 2.0;
    expl_temp.f304.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f304.re = 0.0;
    expl_temp.f304.im = c_tmp / 2.0;
  } else {
    expl_temp.f304.re = c_c_tmp / 2.0;
    expl_temp.f304.im = c_tmp / 2.0;
  }

  c_tmp = ct->f442 * ct->f471 * ct->f493;
  c_c_tmp = c_tmp * ct->f175.re;
  c_tmp *= ct->f175.im;
  if (c_tmp == 0.0) {
    expl_temp.f303.re = c_c_tmp / 2.0;
    expl_temp.f303.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f303.re = 0.0;
    expl_temp.f303.im = c_tmp / 2.0;
  } else {
    expl_temp.f303.re = c_c_tmp / 2.0;
    expl_temp.f303.im = c_tmp / 2.0;
  }

  c_tmp = ct->f451 * ct->f462 * ct->f493;
  c_c_tmp = c_tmp * ct->f175.re;
  c_tmp *= ct->f175.im;
  if (c_tmp == 0.0) {
    expl_temp.f302.re = c_c_tmp / 2.0;
    expl_temp.f302.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f302.re = 0.0;
    expl_temp.f302.im = c_tmp / 2.0;
  } else {
    expl_temp.f302.re = c_c_tmp / 2.0;
    expl_temp.f302.im = c_tmp / 2.0;
  }

  c_tmp = ct->f450 * ct->f470 * ct->f491;
  c_c_tmp = c_tmp * ct->f174.re;
  c_tmp *= ct->f174.im;
  if (c_tmp == 0.0) {
    expl_temp.f301.re = c_c_tmp / 2.0;
    expl_temp.f301.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f301.re = 0.0;
    expl_temp.f301.im = c_tmp / 2.0;
  } else {
    expl_temp.f301.re = c_c_tmp / 2.0;
    expl_temp.f301.im = c_tmp / 2.0;
  }

  c_tmp = ct->f442 * ct->f462 * ct->f493;
  c_c_tmp = c_tmp * ct->f175.re;
  c_tmp *= ct->f175.im;
  if (c_tmp == 0.0) {
    expl_temp.f300.re = c_c_tmp / 2.0;
    expl_temp.f300.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f300.re = 0.0;
    expl_temp.f300.im = c_tmp / 2.0;
  } else {
    expl_temp.f300.re = c_c_tmp / 2.0;
    expl_temp.f300.im = c_tmp / 2.0;
  }

  c_tmp = ct->f441 * ct->f470 * ct->f491;
  c_c_tmp = c_tmp * ct->f174.re;
  c_tmp *= ct->f174.im;
  if (c_tmp == 0.0) {
    expl_temp.f299.re = c_c_tmp / 2.0;
    expl_temp.f299.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f299.re = 0.0;
    expl_temp.f299.im = c_tmp / 2.0;
  } else {
    expl_temp.f299.re = c_c_tmp / 2.0;
    expl_temp.f299.im = c_tmp / 2.0;
  }

  c_tmp = ct->f450 * ct->f461 * ct->f491;
  c_c_tmp = c_tmp * ct->f174.re;
  c_tmp *= ct->f174.im;
  if (c_tmp == 0.0) {
    expl_temp.f298.re = c_c_tmp / 2.0;
    expl_temp.f298.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f298.re = 0.0;
    expl_temp.f298.im = c_tmp / 2.0;
  } else {
    expl_temp.f298.re = c_c_tmp / 2.0;
    expl_temp.f298.im = c_tmp / 2.0;
  }

  c_tmp = ct->f449 * ct->f469 * ct->f489;
  c_c_tmp = c_tmp * ct->f173.re;
  c_tmp *= ct->f173.im;
  if (c_tmp == 0.0) {
    expl_temp.f297.re = c_c_tmp / 2.0;
    expl_temp.f297.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f297.re = 0.0;
    expl_temp.f297.im = c_tmp / 2.0;
  } else {
    expl_temp.f297.re = c_c_tmp / 2.0;
    expl_temp.f297.im = c_tmp / 2.0;
  }

  c_tmp = ct->f441 * ct->f461 * ct->f491;
  c_c_tmp = c_tmp * ct->f174.re;
  c_tmp *= ct->f174.im;
  if (c_tmp == 0.0) {
    expl_temp.f296.re = c_c_tmp / 2.0;
    expl_temp.f296.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f296.re = 0.0;
    expl_temp.f296.im = c_tmp / 2.0;
  } else {
    expl_temp.f296.re = c_c_tmp / 2.0;
    expl_temp.f296.im = c_tmp / 2.0;
  }

  c_tmp = ct->f440 * ct->f469 * ct->f489;
  c_c_tmp = c_tmp * ct->f173.re;
  c_tmp *= ct->f173.im;
  if (c_tmp == 0.0) {
    expl_temp.f295.re = c_c_tmp / 2.0;
    expl_temp.f295.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f295.re = 0.0;
    expl_temp.f295.im = c_tmp / 2.0;
  } else {
    expl_temp.f295.re = c_c_tmp / 2.0;
    expl_temp.f295.im = c_tmp / 2.0;
  }

  c_tmp = ct->f449 * ct->f460 * ct->f489;
  c_c_tmp = c_tmp * ct->f173.re;
  c_tmp *= ct->f173.im;
  if (c_tmp == 0.0) {
    expl_temp.f294.re = c_c_tmp / 2.0;
    expl_temp.f294.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f294.re = 0.0;
    expl_temp.f294.im = c_tmp / 2.0;
  } else {
    expl_temp.f294.re = c_c_tmp / 2.0;
    expl_temp.f294.im = c_tmp / 2.0;
  }

  c_tmp = ct->f448 * ct->f468 * ct->f487;
  c_c_tmp = c_tmp * ct->f172.re;
  c_tmp *= ct->f172.im;
  if (c_tmp == 0.0) {
    expl_temp.f293.re = c_c_tmp / 2.0;
    expl_temp.f293.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f293.re = 0.0;
    expl_temp.f293.im = c_tmp / 2.0;
  } else {
    expl_temp.f293.re = c_c_tmp / 2.0;
    expl_temp.f293.im = c_tmp / 2.0;
  }

  c_tmp = ct->f440 * ct->f460 * ct->f489;
  c_c_tmp = c_tmp * ct->f173.re;
  c_tmp *= ct->f173.im;
  if (c_tmp == 0.0) {
    expl_temp.f292.re = c_c_tmp / 2.0;
    expl_temp.f292.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f292.re = 0.0;
    expl_temp.f292.im = c_tmp / 2.0;
  } else {
    expl_temp.f292.re = c_c_tmp / 2.0;
    expl_temp.f292.im = c_tmp / 2.0;
  }

  c_tmp = ct->f439 * ct->f468 * ct->f487;
  c_c_tmp = c_tmp * ct->f172.re;
  c_tmp *= ct->f172.im;
  if (c_tmp == 0.0) {
    expl_temp.f291.re = c_c_tmp / 2.0;
    expl_temp.f291.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f291.re = 0.0;
    expl_temp.f291.im = c_tmp / 2.0;
  } else {
    expl_temp.f291.re = c_c_tmp / 2.0;
    expl_temp.f291.im = c_tmp / 2.0;
  }

  c_tmp = ct->f448 * ct->f459 * ct->f487;
  c_c_tmp = c_tmp * ct->f172.re;
  c_tmp *= ct->f172.im;
  if (c_tmp == 0.0) {
    expl_temp.f290.re = c_c_tmp / 2.0;
    expl_temp.f290.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f290.re = 0.0;
    expl_temp.f290.im = c_tmp / 2.0;
  } else {
    expl_temp.f290.re = c_c_tmp / 2.0;
    expl_temp.f290.im = c_tmp / 2.0;
  }

  c_tmp = ct->f447 * ct->f467 * ct->f485;
  c_c_tmp = c_tmp * ct->f171.re;
  c_tmp *= ct->f171.im;
  if (c_tmp == 0.0) {
    expl_temp.f289.re = c_c_tmp / 2.0;
    expl_temp.f289.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f289.re = 0.0;
    expl_temp.f289.im = c_tmp / 2.0;
  } else {
    expl_temp.f289.re = c_c_tmp / 2.0;
    expl_temp.f289.im = c_tmp / 2.0;
  }

  c_tmp = ct->f439 * ct->f459 * ct->f487;
  c_c_tmp = c_tmp * ct->f172.re;
  c_tmp *= ct->f172.im;
  if (c_tmp == 0.0) {
    expl_temp.f288.re = c_c_tmp / 2.0;
    expl_temp.f288.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f288.re = 0.0;
    expl_temp.f288.im = c_tmp / 2.0;
  } else {
    expl_temp.f288.re = c_c_tmp / 2.0;
    expl_temp.f288.im = c_tmp / 2.0;
  }

  c_tmp = ct->f438 * ct->f467 * ct->f485;
  c_c_tmp = c_tmp * ct->f171.re;
  c_tmp *= ct->f171.im;
  if (c_tmp == 0.0) {
    expl_temp.f287.re = c_c_tmp / 2.0;
    expl_temp.f287.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f287.re = 0.0;
    expl_temp.f287.im = c_tmp / 2.0;
  } else {
    expl_temp.f287.re = c_c_tmp / 2.0;
    expl_temp.f287.im = c_tmp / 2.0;
  }

  c_tmp = ct->f447 * ct->f458 * ct->f485;
  c_c_tmp = c_tmp * ct->f171.re;
  c_tmp *= ct->f171.im;
  if (c_tmp == 0.0) {
    expl_temp.f286.re = c_c_tmp / 2.0;
    expl_temp.f286.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f286.re = 0.0;
    expl_temp.f286.im = c_tmp / 2.0;
  } else {
    expl_temp.f286.re = c_c_tmp / 2.0;
    expl_temp.f286.im = c_tmp / 2.0;
  }

  c_tmp = ct->f446 * ct->f465 * ct->f481;
  c_c_tmp = c_tmp * ct->f170.re;
  c_tmp *= ct->f170.im;
  if (c_tmp == 0.0) {
    expl_temp.f285.re = c_c_tmp / 2.0;
    expl_temp.f285.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f285.re = 0.0;
    expl_temp.f285.im = c_tmp / 2.0;
  } else {
    expl_temp.f285.re = c_c_tmp / 2.0;
    expl_temp.f285.im = c_tmp / 2.0;
  }

  c_tmp = ct->f438 * ct->f458 * ct->f485;
  c_c_tmp = c_tmp * ct->f171.re;
  c_tmp *= ct->f171.im;
  if (c_tmp == 0.0) {
    expl_temp.f284.re = c_c_tmp / 2.0;
    expl_temp.f284.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f284.re = 0.0;
    expl_temp.f284.im = c_tmp / 2.0;
  } else {
    expl_temp.f284.re = c_c_tmp / 2.0;
    expl_temp.f284.im = c_tmp / 2.0;
  }

  c_tmp = ct->f437 * ct->f465 * ct->f481;
  c_c_tmp = c_tmp * ct->f170.re;
  c_tmp *= ct->f170.im;
  if (c_tmp == 0.0) {
    expl_temp.f283.re = c_c_tmp / 2.0;
    expl_temp.f283.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f283.re = 0.0;
    expl_temp.f283.im = c_tmp / 2.0;
  } else {
    expl_temp.f283.re = c_c_tmp / 2.0;
    expl_temp.f283.im = c_tmp / 2.0;
  }

  c_tmp = ct->f446 * ct->f457 * ct->f481;
  c_c_tmp = c_tmp * ct->f170.re;
  c_tmp *= ct->f170.im;
  if (c_tmp == 0.0) {
    expl_temp.f282.re = c_c_tmp / 2.0;
    expl_temp.f282.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f282.re = 0.0;
    expl_temp.f282.im = c_tmp / 2.0;
  } else {
    expl_temp.f282.re = c_c_tmp / 2.0;
    expl_temp.f282.im = c_tmp / 2.0;
  }

  c_tmp = ct->f445 * ct->f464 * ct->f479;
  c_c_tmp = c_tmp * ct->f169.re;
  c_tmp *= ct->f169.im;
  if (c_tmp == 0.0) {
    expl_temp.f281.re = c_c_tmp / 2.0;
    expl_temp.f281.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f281.re = 0.0;
    expl_temp.f281.im = c_tmp / 2.0;
  } else {
    expl_temp.f281.re = c_c_tmp / 2.0;
    expl_temp.f281.im = c_tmp / 2.0;
  }

  c_tmp = ct->f437 * ct->f457 * ct->f481;
  c_c_tmp = c_tmp * ct->f170.re;
  c_tmp *= ct->f170.im;
  if (c_tmp == 0.0) {
    expl_temp.f280.re = c_c_tmp / 2.0;
    expl_temp.f280.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f280.re = 0.0;
    expl_temp.f280.im = c_tmp / 2.0;
  } else {
    expl_temp.f280.re = c_c_tmp / 2.0;
    expl_temp.f280.im = c_tmp / 2.0;
  }

  c_tmp = ct->f436 * ct->f464 * ct->f479;
  c_c_tmp = c_tmp * ct->f169.re;
  c_tmp *= ct->f169.im;
  if (c_tmp == 0.0) {
    expl_temp.f279.re = c_c_tmp / 2.0;
    expl_temp.f279.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f279.re = 0.0;
    expl_temp.f279.im = c_tmp / 2.0;
  } else {
    expl_temp.f279.re = c_c_tmp / 2.0;
    expl_temp.f279.im = c_tmp / 2.0;
  }

  c_tmp = ct->f445 * ct->f456 * ct->f479;
  c_c_tmp = c_tmp * ct->f169.re;
  c_tmp *= ct->f169.im;
  if (c_tmp == 0.0) {
    expl_temp.f278.re = c_c_tmp / 2.0;
    expl_temp.f278.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f278.re = 0.0;
    expl_temp.f278.im = c_tmp / 2.0;
  } else {
    expl_temp.f278.re = c_c_tmp / 2.0;
    expl_temp.f278.im = c_tmp / 2.0;
  }

  c_tmp = ct->f436 * ct->f456 * ct->f479;
  c_c_tmp = c_tmp * ct->f169.re;
  c_tmp *= ct->f169.im;
  if (c_tmp == 0.0) {
    expl_temp.f277.re = c_c_tmp / 2.0;
    expl_temp.f277.im = 0.0;
  } else if (c_c_tmp == 0.0) {
    expl_temp.f277.re = 0.0;
    expl_temp.f277.im = c_tmp / 2.0;
  } else {
    expl_temp.f277.re = c_c_tmp / 2.0;
    expl_temp.f277.im = c_tmp / 2.0;
  }

  expl_temp.f276 = ct->f218;
  expl_temp.f275 = ct->f217;
  expl_temp.f274 = ct->f216;
  expl_temp.f273 = ct->f215;
  expl_temp.f272 = ct->f214;
  expl_temp.f271 = ct->f212;
  expl_temp.f270 = ct->f211;
  expl_temp.f269 = ct->f210;
  expl_temp.f268 = ct->f209;
  expl_temp.f267 = ct->f208;
  expl_temp.f266 = ct->f207;
  expl_temp.f265 = ct->f206;
  expl_temp.f264 = ct->f205;
  expl_temp.f263 = ct->f204;
  expl_temp.f262 = ct->f203;
  expl_temp.f261 = ct->f201;
  expl_temp.f260 = ct->f200;
  expl_temp.f259 = ct->f199;
  expl_temp.f258 = ct->f198;
  expl_temp.f257 = ct->f197;
  expl_temp.f256 = ct->f196;
  expl_temp.f255 = ct->f195;
  expl_temp.f254 = ct->f194;
  expl_temp.f253 = ct->f193;
  expl_temp.f252 = ct->f192;
  expl_temp.f251 = ct->f191;
  expl_temp.f250 = ct->f189;
  expl_temp.f249 = ct->f188;
  expl_temp.f248 = ct->f187;
  expl_temp.f247 = ct->f186;
  expl_temp.f246 = ct->f185;
  expl_temp.f245 = ct->f184;
  expl_temp.f244 = ct->f183;
  expl_temp.f243 = ct->f182;
  expl_temp.f242 = ct->f181;
  expl_temp.f241 = ct->f180;
  expl_temp.f240 = ct->f178;
  expl_temp.f239 = ct->f177;
  expl_temp.f238 = ct->f176;
  expl_temp.f237 = ct->f175;
  expl_temp.f236 = ct->f174;
  expl_temp.f235 = ct->f173;
  expl_temp.f234 = ct->f172;
  expl_temp.f233 = ct->f171;
  expl_temp.f232 = ct->f170;
  expl_temp.f231 = ct->f169;
  c_tmp = ct->f510 * ct->f435 * ct->f497;
  expl_temp.f230 = c_tmp * -20.0;
  expl_temp.f229 = ct->f509 * ct->f435 * ct->f497 * -12.0;
  expl_temp.f228 = ct->f510 * ct->f434 * ct->f497 * -20.0;
  expl_temp.f227 = ct->f509 * ct->f434 * ct->f497 * -12.0;
  expl_temp.f226 = ct->f477 * ct->f432 * ct->f494 * -20.0;
  expl_temp.f225 = ct->f466 * ct->f432 * ct->f494 * -12.0;
  expl_temp.f224 = ct->f455 * ct->f431 * ct->f492 * -20.0;
  expl_temp.f223 = ct->f422 * ct->f431 * ct->f492 * -12.0;
  expl_temp.f222 = ct->f411 * ct->f430 * ct->f490 * -20.0;
  expl_temp.f221 = ct->f477 * ct->f424 * ct->f494 * -20.0;
  expl_temp.f220 = ct->f378 * ct->f430 * ct->f490 * -12.0;
  expl_temp.f219 = ct->f466 * ct->f424 * ct->f494 * -12.0;
  expl_temp.f218 = ct->f367 * ct->f429 * ct->f488 * -20.0;
  expl_temp.f217 = ct->f455 * ct->f423 * ct->f492 * -20.0;
  expl_temp.f216 = ct->f363 * ct->f429 * ct->f488 * -12.0;
  expl_temp.f215 = ct->f422 * ct->f423 * ct->f492 * -12.0;
  expl_temp.f214 = ct->f362 * ct->f428 * ct->f486 * -20.0;
  expl_temp.f213 = ct->f411 * ct->f421 * ct->f490 * -20.0;
  expl_temp.f212 = ct->f354 * ct->f428 * ct->f486 * -12.0;
  expl_temp.f211 = ct->f378 * ct->f421 * ct->f490 * -12.0;
  expl_temp.f210 = ct->f345 * ct->f427 * ct->f482 * -20.0;
  expl_temp.f209 = ct->f367 * ct->f420 * ct->f488 * -20.0;
  expl_temp.f208 = ct->f336 * ct->f427 * ct->f482 * -12.0;
  expl_temp.f207 = ct->f363 * ct->f420 * ct->f488 * -12.0;
  expl_temp.f206 = ct->f335 * ct->f426 * ct->f480 * -20.0;
  expl_temp.f205 = ct->f362 * ct->f419 * ct->f486 * -20.0;
  expl_temp.f204 = ct->f333 * ct->f426 * ct->f480 * -12.0;
  expl_temp.f203 = ct->f354 * ct->f419 * ct->f486 * -12.0;
  expl_temp.f202 = ct->f330 * ct->f425 * ct->f478 * -20.0;
  expl_temp.f201 = ct->f345 * ct->f418 * ct->f482 * -20.0;
  expl_temp.f200 = ct->f328 * ct->f425 * ct->f478 * -12.0;
  expl_temp.f199 = ct->f336 * ct->f418 * ct->f482 * -12.0;
  expl_temp.f198 = ct->f335 * ct->f417 * ct->f480 * -20.0;
  expl_temp.f197 = ct->f333 * ct->f417 * ct->f480 * -12.0;
  expl_temp.f196 = ct->f330 * ct->f416 * ct->f478 * -20.0;
  expl_temp.f195 = ct->f328 * ct->f416 * ct->f478 * -12.0;
  expl_temp.f194 = ct->f285 * ct->f435 * ct->f497;
  expl_temp.f193 = ct->f511 * ct->f435 * ct->f497 * 10.0;
  expl_temp.f192 = ct->f284 * ct->f435 * ct->f497;
  expl_temp.f191 = c_tmp * 8.0;
  expl_temp.f190 = ct->f285 * ct->f434 * ct->f497;
  expl_temp.f189 = ct->f511 * ct->f434 * ct->f497 * 10.0;
  expl_temp.f188 = ct->f284 * ct->f434 * ct->f497;
  expl_temp.f187 = ct->f160;
  expl_temp.f186 = ct->f159;
  expl_temp.f185 = ct->f157;
  expl_temp.f184 = ct->f156;
  expl_temp.f183 = ct->f155;
  expl_temp.f182 = ct->f154;
  expl_temp.f181 = ct->f153;
  expl_temp.f180 = ct->f152;
  expl_temp.f179 = ct->f151;
  expl_temp.f178 = ct->f150;
  expl_temp.f177 = ct->f149;
  expl_temp.f176 = ct->f148;
  expl_temp.f175 = ct->f147;
  expl_temp.f174 = ct->f146;
  expl_temp.f173 = ct->f145;
  expl_temp.f172 = ct->f144;
  expl_temp.f171 = ct->f143;
  expl_temp.f170 = ct->f142;
  expl_temp.f169 = ct->f141;
  expl_temp.f168 = ct->f140;
  expl_temp.f167 = ct->f139;
  expl_temp.f166 = ct->f138;
  expl_temp.f165 = ct->f137;
  expl_temp.f164 = ct->f136;
  expl_temp.f163 = ct->f135;
  expl_temp.f162 = ct->f134;
  expl_temp.f161 = ct->f133;
  expl_temp.f160 = ct->f132;
  expl_temp.f159 = ct->f131;
  expl_temp.f158 = ct->f130;
  expl_temp.f157 = ct->f129;
  expl_temp.f156 = ct->f128;
  expl_temp.f155 = ct->f126;
  expl_temp.f154 = ct->f125;
  expl_temp.f153 = ct->f124;
  expl_temp.f152 = ct->f123;
  expl_temp.f151 = ct->f122;
  expl_temp.f150 = ct->f121;
  expl_temp.f149 = ct->f120;
  expl_temp.f148 = ct->f119;
  expl_temp.f147 = ct->f118;
  expl_temp.f146 = ct->f117;
  expl_temp.f145 = ct->f116;
  expl_temp.f144 = ct->f115;
  expl_temp.f143 = ct->f114;
  expl_temp.f142 = ct->f113;
  expl_temp.f141 = ct->f112;
  expl_temp.f140 = ct->f111;
  expl_temp.f139 = ct->f110;
  expl_temp.f138 = ct->f109;
  expl_temp.f137 = ct->f108;
  expl_temp.f136 = ct->f107;
  expl_temp.f135 = ct->f106;
  expl_temp.f134 = ct->f105;
  expl_temp.f133 = ct->f104;
  expl_temp.f132 = ct->f103;
  expl_temp.f131 = ct->f102;
  expl_temp.f130 = ct->f101;
  expl_temp.f129 = ct->f100;
  expl_temp.f128 = ct->f99;
  expl_temp.f127 = ct->f98;
  expl_temp.f126 = ct->f97;
  expl_temp.f125 = ct->f95;
  expl_temp.f124 = ct->f94;
  expl_temp.f123 = ct->f93;
  expl_temp.f122 = ct->f92;
  expl_temp.f121 = ct->f91;
  expl_temp.f120 = ct->f90;
  expl_temp.f119 = ct->f89;
  expl_temp.f118 = ct->f88;
  expl_temp.f117 = ct->f87;
  expl_temp.f116 = ct->f86;
  expl_temp.f115 = ct->f85;
  expl_temp.f114 = ct->f84;
  expl_temp.f113 = ct->f83;
  expl_temp.f112 = ct->f82;
  expl_temp.f111 = ct->f81;
  expl_temp.f110 = ct->f80;
  expl_temp.f109 = ct->f79;
  expl_temp.f108 = ct->f78;
  expl_temp.f107 = ct->f77;
  expl_temp.f106 = ct->f76;
  expl_temp.f105 = ct->f75;
  expl_temp.f104 = ct->f74;
  expl_temp.f103 = ct->f668 * ct->f677;
  expl_temp.f102 = ct->f666 * ct->f677;
  expl_temp.f101 = ct->f664 * ct->f676;
  expl_temp.f100 = ct->f662 * ct->f675;
  expl_temp.f99 = ct->f660 * ct->f674;
  expl_temp.f98 = ct->f658 * ct->f676;
  expl_temp.f97 = ct->f656 * ct->f673;
  expl_temp.f96 = ct->f654 * ct->f675;
  expl_temp.f95 = ct->f652 * ct->f672;
  expl_temp.f94 = ct->f650 * ct->f674;
  expl_temp.f93 = ct->f648 * ct->f671;
  expl_temp.f92 = ct->f646 * ct->f673;
  expl_temp.f91 = ct->f644 * ct->f670;
  expl_temp.f90 = ct->f642 * ct->f672;
  expl_temp.f89 = ct->f640 * ct->f669;
  expl_temp.f88 = ct->f638 * ct->f671;
  expl_temp.f87 = ct->f636 * ct->f670;
  expl_temp.f86 = ct->f634 * ct->f669;
  expl_temp.f85 = ct->f511 * ct->f595 * -5.0;
  expl_temp.f84 = ct->f510 * ct->f595 * -4.0;
  expl_temp.f83 = ct->f509 * ct->f595 * -3.0;
  expl_temp.f82 = ct->f511 * ct->f594 * -5.0;
  expl_temp.f81 = ct->f510 * ct->f594 * -4.0;
  expl_temp.f80 = ct->f509 * ct->f594 * -3.0;
  expl_temp.f79 = ct->f495 * ct->f593 * -5.0;
  expl_temp.f78 = ct->f477 * ct->f593 * -4.0;
  expl_temp.f77 = ct->f466 * ct->f593 * -3.0;
  expl_temp.f76 = ct->f476 * ct->f592 * -5.0;
  expl_temp.f75 = ct->f455 * ct->f592 * -4.0;
  expl_temp.f74 = ct->f422 * ct->f592 * -3.0;
  expl_temp.f73 = ct->f495 * ct->f590 * -5.0;
  expl_temp.f72 = ct->f444 * ct->f591 * -5.0;
  expl_temp.f71 = ct->f477 * ct->f590 * -4.0;
  expl_temp.f70 = ct->f411 * ct->f591 * -4.0;
  expl_temp.f69 = ct->f466 * ct->f590 * -3.0;
  expl_temp.f68 = ct->f378 * ct->f591 * -3.0;
  expl_temp.f67 = ct->f476 * ct->f588 * -5.0;
  expl_temp.f66 = ct->f400 * ct->f589 * -5.0;
  expl_temp.f65 = ct->f455 * ct->f588 * -4.0;
  expl_temp.f64 = ct->f367 * ct->f589 * -4.0;
  expl_temp.f63 = ct->f422 * ct->f588 * -3.0;
  expl_temp.f62 = ct->f363 * ct->f589 * -3.0;
  expl_temp.f61 = ct->f444 * ct->f586 * -5.0;
  expl_temp.f60 = ct->f365 * ct->f587 * -5.0;
  expl_temp.f59 = ct->f411 * ct->f586 * -4.0;
  expl_temp.f58 = ct->f362 * ct->f587 * -4.0;
  expl_temp.f57 = ct->f378 * ct->f586 * -3.0;
  expl_temp.f56 = ct->f354 * ct->f587 * -3.0;
  expl_temp.f55 = ct->f400 * ct->f583 * -5.0;
  expl_temp.f54 = ct->f361 * ct->f585 * -5.0;
  expl_temp.f53 = ct->f367 * ct->f583 * -4.0;
  expl_temp.f52 = ct->f345 * ct->f585 * -4.0;
  expl_temp.f51 = ct->f363 * ct->f583 * -3.0;
  expl_temp.f50 = ct->f336 * ct->f585 * -3.0;
  expl_temp.f49 = ct->f365 * ct->f581 * -5.0;
  expl_temp.f48 = ct->f340 * ct->f582 * -5.0;
  expl_temp.f47 = ct->f362 * ct->f581 * -4.0;
  expl_temp.f46 = ct->f335 * ct->f582 * -4.0;
  expl_temp.f45 = ct->f354 * ct->f581 * -3.0;
  expl_temp.f44 = ct->f333 * ct->f582 * -3.0;
  expl_temp.f43 = ct->f361 * ct->f579 * -5.0;
  expl_temp.f42 = ct->f334 * ct->f580 * -5.0;
  expl_temp.f41 = ct->f345 * ct->f579 * -4.0;
  expl_temp.f40 = ct->f330 * ct->f580 * -4.0;
  expl_temp.f39 = ct->f336 * ct->f579 * -3.0;
  expl_temp.f38 = ct->f328 * ct->f580 * -3.0;
  expl_temp.f37 = ct->f340 * ct->f578 * -5.0;
  expl_temp.f36 = ct->f335 * ct->f578 * -4.0;
  expl_temp.f35 = ct->f333 * ct->f578 * -3.0;
  expl_temp.f34 = ct->f334 * ct->f577 * -5.0;
  expl_temp.f33 = ct->f330 * ct->f577 * -4.0;
  expl_temp.f32 = ct->f328 * ct->f577 * -3.0;
  expl_temp.f31 = ct->f72;
  expl_temp.f30 = ct->f71;
  expl_temp.f29 = ct->f70;
  expl_temp.f28 = ct->f69;
  expl_temp.f27 = ct->f68;
  expl_temp.f26 = ct->f67;
  expl_temp.f25 = ct->f66;
  expl_temp.f24 = ct->f65;
  expl_temp.f23 = ct->f64;
  expl_temp.f22 = ct->f63;
  expl_temp.f21 = ct->f62;
  expl_temp.f20 = ct->f61;
  expl_temp.f19 = ct->f60;
  expl_temp.f18 = ct->f59;
  expl_temp.f17 = ct->f58;
  expl_temp.f16 = ct->f57;
  expl_temp.f15 = ct->f56;
  expl_temp.f14 = ct->f55;
  expl_temp.f13 = ct->f261 * ct->f595;
  expl_temp.f12 = ct->f260 * ct->f595;
  expl_temp.f11 = ct->f259 * ct->f595;
  expl_temp.f10 = ct->f261 * ct->f594;
  expl_temp.f9 = ct->f260 * ct->f594;
  expl_temp.f8 = ct->f259 * ct->f594;
  expl_temp.f7 = ct->f258 * ct->f593;
  expl_temp.f6 = ct->f256 * ct->f593;
  expl_temp.f5 = ct->f251 * ct->f593;
  expl_temp.f4 = ct->f54;
  expl_temp.f3 = ct->f53;
  expl_temp.f2 = ct->f41;
  expl_temp.f1 = ct->f17;
  ft_3(&expl_temp, c_gradient, ceq_gradient_data, ceq_gradient_size);
  c_gradient_size[0] = 128;
  c_gradient_size[1] = 18;
  memcpy(&c_gradient_data[0], &c_gradient[0], 2304U * sizeof(creal_T));
}

/*
 * Arguments    : const cell_16 *ct
 *                creal_T c_gradient[2304]
 *                double ceq_gradient_data[]
 *                int ceq_gradient_size[2]
 * Return Type  : void
 */
static void ft_3(const cell_16 *ct, creal_T c_gradient[2304], double
                 ceq_gradient_data[], int ceq_gradient_size[2])
{
  static const signed char iv[131] = { 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double dv[324];
  double ab_ct_im_tmp;
  double ab_ct_re_tmp;
  double b_ct_im_tmp;
  double b_ct_re_tmp;
  double bb_ct_im_tmp;
  double bb_ct_re_tmp;
  double c_ct_im_tmp;
  double c_ct_re_tmp;
  double cb_ct_im_tmp;
  double cb_ct_re_tmp;
  double ct_im;
  double ct_im_tmp;
  double ct_re;
  double ct_re_tmp;
  double d_ct_im_tmp;
  double d_ct_re_tmp;
  double db_ct_im_tmp;
  double db_ct_re_tmp;
  double e_ct_im_tmp;
  double e_ct_re_tmp;
  double eb_ct_im_tmp;
  double eb_ct_re_tmp;
  double f_ct_im_tmp;
  double f_ct_re_tmp;
  double fb_ct_im_tmp;
  double fb_ct_re_tmp;
  double g_ct_im_tmp;
  double g_ct_re_tmp;
  double gb_ct_im_tmp;
  double gb_ct_re_tmp;
  double h_ct_im_tmp;
  double h_ct_re_tmp;
  double hb_ct_im_tmp;
  double hb_ct_re_tmp;
  double i_ct_im_tmp;
  double i_ct_re_tmp;
  double ib_ct_im_tmp;
  double ib_ct_re_tmp;
  double j_ct_im_tmp;
  double j_ct_re_tmp;
  double jb_ct_im_tmp;
  double jb_ct_re_tmp;
  double k_ct_im_tmp;
  double k_ct_re_tmp;
  double kb_ct_im_tmp;
  double kb_ct_re_tmp;
  double l_ct_im_tmp;
  double l_ct_re_tmp;
  double m_ct_im_tmp;
  double m_ct_re_tmp;
  double n_ct_im_tmp;
  double n_ct_re_tmp;
  double o_ct_im_tmp;
  double o_ct_re_tmp;
  double p_ct_im_tmp;
  double p_ct_re_tmp;
  double q_ct_im_tmp;
  double q_ct_re_tmp;
  double r_ct_im_tmp;
  double r_ct_re_tmp;
  double s_ct_im_tmp;
  double s_ct_re_tmp;
  double t1603_im;
  double t1603_re;
  double t1604_im;
  double t1604_re;
  double t1605_im;
  double t1605_re;
  double t1606_im;
  double t1606_re;
  double t1607_im;
  double t1607_re;
  double t1608_im;
  double t1608_re;
  double t1609_im;
  double t1609_re;
  double t1610_im;
  double t1610_re;
  double t1611_im;
  double t1611_re;
  double t1612_im;
  double t1612_re;
  double t1613_im;
  double t1613_re;
  double t1614_im;
  double t1614_re;
  double t1615_im;
  double t1615_re;
  double t1616_im;
  double t1616_re;
  double t1617_im;
  double t1617_re;
  double t1618_im;
  double t1618_re;
  double t1619_im;
  double t1619_re;
  double t1620_im;
  double t1620_re;
  double t1621_im;
  double t1621_re;
  double t1622_im;
  double t1622_re;
  double t1623_im;
  double t1623_re;
  double t1624_im;
  double t1624_re;
  double t1625_im;
  double t1625_re;
  double t1626_im;
  double t1626_re;
  double t1627_im;
  double t1627_re;
  double t1628_im;
  double t1628_re;
  double t1629_im;
  double t1629_re;
  double t1630_im;
  double t1630_re;
  double t1631_im;
  double t1631_re;
  double t1632_im;
  double t1632_re;
  double t1633_im;
  double t1633_re;
  double t1634_im;
  double t1634_re;
  double t1635_im;
  double t1635_re;
  double t1636_im;
  double t1636_re;
  double t1637_im;
  double t1637_re;
  double t1638_im;
  double t1638_re;
  double t1639_im;
  double t1639_re;
  double t1640_im;
  double t1640_re;
  double t1641_im;
  double t1641_re;
  double t1642_im;
  double t1642_re;
  double t1643_im;
  double t1643_re;
  double t1644_im;
  double t1644_re;
  double t1645_im;
  double t1645_re;
  double t1646_im;
  double t1646_re;
  double t1647_im;
  double t1647_re;
  double t1648_im;
  double t1648_re;
  double t1649_im;
  double t1649_re;
  double t1650_im;
  double t1650_re;
  double t1651_im;
  double t1651_re;
  double t1652_im;
  double t1652_re;
  double t1653_im;
  double t1653_re;
  double t1654_im;
  double t1654_re;
  double t1655_im;
  double t1655_re;
  double t1656_im;
  double t1656_re;
  double t1657_im;
  double t1657_re;
  double t1658_im;
  double t1658_re;
  double t1659_im;
  double t1659_re;
  double t1660_im;
  double t1660_re;
  double t1661_im;
  double t1661_re;
  double t1662_im;
  double t1662_re;
  double t1663_im;
  double t1663_re;
  double t1664_im;
  double t1664_re;
  double t1665_im;
  double t1665_re;
  double t1666_im;
  double t1666_re;
  double t1667_im;
  double t1667_re;
  double t1668_im;
  double t1668_re;
  double t1669_im;
  double t1669_re;
  double t1670_im;
  double t1670_re;
  double t1671_im;
  double t1671_re;
  double t1672_im;
  double t1672_re;
  double t1673_im;
  double t1673_re;
  double t1674_im;
  double t1674_re;
  double t1675_im;
  double t1675_re;
  double t1676_im;
  double t1676_re;
  double t1677_im;
  double t1677_re;
  double t1678_im;
  double t1678_re;
  double t1679_im;
  double t1679_re;
  double t1680_im;
  double t1680_re;
  double t1681_im;
  double t1681_re;
  double t1682_im;
  double t1682_re;
  double t1683_im;
  double t1683_re;
  double t1684_im;
  double t1684_re;
  double t1685_im;
  double t1685_re;
  double t1686_im;
  double t1686_re;
  double t1687_im;
  double t1687_re;
  double t1688_im;
  double t1688_re;
  double t1689_im;
  double t1689_re;
  double t1690_im;
  double t1690_re;
  double t1691_im;
  double t1691_re;
  double t1692_im;
  double t1692_re;
  double t1693_im;
  double t1693_re;
  double t1694_im;
  double t1694_re;
  double t1695_im;
  double t1695_re;
  double t1696_im;
  double t1696_re;
  double t1697_im;
  double t1697_re;
  double t1698_im;
  double t1698_re;
  double t1699_im;
  double t1699_re;
  double t1700_im;
  double t1700_re;
  double t1701_im;
  double t1701_re;
  double t1702_im;
  double t1702_re;
  double t1703_im;
  double t1703_re;
  double t1704_im;
  double t1704_re;
  double t1705_im;
  double t1705_re;
  double t1706_im;
  double t1706_re;
  double t1707_im;
  double t1707_re;
  double t1708_im;
  double t1708_re;
  double t1709_im;
  double t1709_re;
  double t1710_im;
  double t1710_re;
  double t1711_im;
  double t1711_re;
  double t1712_im;
  double t1712_re;
  double t1713_im;
  double t1713_re;
  double t1714_im;
  double t1714_re;
  double t1715_im;
  double t1715_re;
  double t1716_im;
  double t1716_re;
  double t1717_im;
  double t1717_re;
  double t1718_im;
  double t1718_re;
  double t1719_im;
  double t1719_re;
  double t1720_im;
  double t1720_re;
  double t1721_im;
  double t1721_re;
  double t1722_im;
  double t1722_re;
  double t1723_im;
  double t1723_re;
  double t1724_im;
  double t1724_re;
  double t1725_im;
  double t1725_re;
  double t1726_im;
  double t1726_re;
  double t1727_im;
  double t1727_re;
  double t1728_im;
  double t1728_re;
  double t1729_im;
  double t1729_re;
  double t1730_im;
  double t1730_re;
  double t1731_im;
  double t1731_re;
  double t1732_im;
  double t1732_re;
  double t1733_im;
  double t1733_re;
  double t1734_im;
  double t1734_re;
  double t1735_im;
  double t1735_re;
  double t1736_im;
  double t1736_re;
  double t1737_im;
  double t1737_re;
  double t1738_im;
  double t1738_re;
  double t1739_im;
  double t1739_re;
  double t1740_im;
  double t1740_re;
  double t1741_im;
  double t1741_re;
  double t1742_im;
  double t1742_re;
  double t1743_im;
  double t1743_re;
  double t1744_im;
  double t1744_re;
  double t1745_im;
  double t1745_re;
  double t1746_im;
  double t1746_re;
  double t_ct_im_tmp;
  double t_ct_re_tmp;
  double u_ct_im_tmp;
  double u_ct_re_tmp;
  double v_ct_im_tmp;
  double v_ct_re_tmp;
  double w_ct_im_tmp;
  double w_ct_re_tmp;
  double x_ct_im_tmp;
  double x_ct_re_tmp;
  double y_ct_im_tmp;
  double y_ct_re_tmp;
  int i;
  t1691_re = ct->f599 * ct->f619 * ct->f664;
  ct_re_tmp = t1691_re * ct->f231.re;
  ct_im_tmp = t1691_re * ct->f231.im;
  ct_re = ct_re_tmp * ct->f240.re - ct_im_tmp * ct->f240.im;
  ct_im = ct_re_tmp * ct->f240.im + ct_im_tmp * ct->f240.re;
  if (ct_im == 0.0) {
    t1603_re = ct_re / 2.0;
    t1603_im = 0.0;
  } else if (ct_re == 0.0) {
    t1603_re = 0.0;
    t1603_im = ct_im / 2.0;
  } else {
    t1603_re = ct_re / 2.0;
    t1603_im = ct_im / 2.0;
  }

  t1691_re = ct->f608 * ct->f619 * ct->f664;
  b_ct_re_tmp = t1691_re * ct->f231.re;
  b_ct_im_tmp = t1691_re * ct->f231.im;
  ct_re = b_ct_re_tmp * ct->f240.re - b_ct_im_tmp * ct->f240.im;
  ct_im = b_ct_re_tmp * ct->f240.im + b_ct_im_tmp * ct->f240.re;
  if (ct_im == 0.0) {
    t1604_re = ct_re / 2.0;
    t1604_im = 0.0;
  } else if (ct_re == 0.0) {
    t1604_re = 0.0;
    t1604_im = ct_im / 2.0;
  } else {
    t1604_re = ct_re / 2.0;
    t1604_im = ct_im / 2.0;
  }

  t1691_re = ct->f599 * ct->f628 * ct->f664;
  c_ct_re_tmp = t1691_re * ct->f231.re;
  c_ct_im_tmp = t1691_re * ct->f231.im;
  ct_re = c_ct_re_tmp * ct->f240.re - c_ct_im_tmp * ct->f240.im;
  ct_im = c_ct_re_tmp * ct->f240.im + c_ct_im_tmp * ct->f240.re;
  if (ct_im == 0.0) {
    t1605_re = ct_re / 2.0;
    t1605_im = 0.0;
  } else if (ct_re == 0.0) {
    t1605_re = 0.0;
    t1605_im = ct_im / 2.0;
  } else {
    t1605_re = ct_re / 2.0;
    t1605_im = ct_im / 2.0;
  }

  t1691_re = ct->f600 * ct->f620 * ct->f666;
  d_ct_re_tmp = t1691_re * ct->f232.re;
  d_ct_im_tmp = t1691_re * ct->f232.im;
  ct_re = d_ct_re_tmp * ct->f241.re - d_ct_im_tmp * ct->f241.im;
  ct_im = d_ct_re_tmp * ct->f241.im + d_ct_im_tmp * ct->f241.re;
  if (ct_im == 0.0) {
    t1606_re = ct_re / 2.0;
    t1606_im = 0.0;
  } else if (ct_re == 0.0) {
    t1606_re = 0.0;
    t1606_im = ct_im / 2.0;
  } else {
    t1606_re = ct_re / 2.0;
    t1606_im = ct_im / 2.0;
  }

  t1691_re = ct->f608 * ct->f628 * ct->f664;
  e_ct_re_tmp = t1691_re * ct->f231.re;
  e_ct_im_tmp = t1691_re * ct->f231.im;
  ct_re = e_ct_re_tmp * ct->f240.re - e_ct_im_tmp * ct->f240.im;
  ct_im = e_ct_re_tmp * ct->f240.im + e_ct_im_tmp * ct->f240.re;
  if (ct_im == 0.0) {
    t1607_re = ct_re / 2.0;
    t1607_im = 0.0;
  } else if (ct_re == 0.0) {
    t1607_re = 0.0;
    t1607_im = ct_im / 2.0;
  } else {
    t1607_re = ct_re / 2.0;
    t1607_im = ct_im / 2.0;
  }

  t1691_re = ct->f609 * ct->f620 * ct->f666;
  f_ct_re_tmp = t1691_re * ct->f232.re;
  f_ct_im_tmp = t1691_re * ct->f232.im;
  ct_re = f_ct_re_tmp * ct->f241.re - f_ct_im_tmp * ct->f241.im;
  ct_im = f_ct_re_tmp * ct->f241.im + f_ct_im_tmp * ct->f241.re;
  if (ct_im == 0.0) {
    t1608_re = ct_re / 2.0;
    t1608_im = 0.0;
  } else if (ct_re == 0.0) {
    t1608_re = 0.0;
    t1608_im = ct_im / 2.0;
  } else {
    t1608_re = ct_re / 2.0;
    t1608_im = ct_im / 2.0;
  }

  t1691_re = ct->f600 * ct->f629 * ct->f666;
  g_ct_re_tmp = t1691_re * ct->f232.re;
  g_ct_im_tmp = t1691_re * ct->f232.im;
  ct_re = g_ct_re_tmp * ct->f241.re - g_ct_im_tmp * ct->f241.im;
  ct_im = g_ct_re_tmp * ct->f241.im + g_ct_im_tmp * ct->f241.re;
  if (ct_im == 0.0) {
    t1609_re = ct_re / 2.0;
    t1609_im = 0.0;
  } else if (ct_re == 0.0) {
    t1609_re = 0.0;
    t1609_im = ct_im / 2.0;
  } else {
    t1609_re = ct_re / 2.0;
    t1609_im = ct_im / 2.0;
  }

  t1691_re = ct->f601 * ct->f621 * ct->f669;
  h_ct_re_tmp = t1691_re * ct->f233.re;
  h_ct_im_tmp = t1691_re * ct->f233.im;
  ct_re = h_ct_re_tmp * ct->f242.re - h_ct_im_tmp * ct->f242.im;
  ct_im = h_ct_re_tmp * ct->f242.im + h_ct_im_tmp * ct->f242.re;
  if (ct_im == 0.0) {
    t1610_re = ct_re / 2.0;
    t1610_im = 0.0;
  } else if (ct_re == 0.0) {
    t1610_re = 0.0;
    t1610_im = ct_im / 2.0;
  } else {
    t1610_re = ct_re / 2.0;
    t1610_im = ct_im / 2.0;
  }

  t1691_re = ct->f609 * ct->f629 * ct->f666;
  i_ct_re_tmp = t1691_re * ct->f232.re;
  i_ct_im_tmp = t1691_re * ct->f232.im;
  ct_re = i_ct_re_tmp * ct->f241.re - i_ct_im_tmp * ct->f241.im;
  ct_im = i_ct_re_tmp * ct->f241.im + i_ct_im_tmp * ct->f241.re;
  if (ct_im == 0.0) {
    t1611_re = ct_re / 2.0;
    t1611_im = 0.0;
  } else if (ct_re == 0.0) {
    t1611_re = 0.0;
    t1611_im = ct_im / 2.0;
  } else {
    t1611_re = ct_re / 2.0;
    t1611_im = ct_im / 2.0;
  }

  t1691_re = ct->f610 * ct->f621 * ct->f669;
  j_ct_re_tmp = t1691_re * ct->f233.re;
  j_ct_im_tmp = t1691_re * ct->f233.im;
  ct_re = j_ct_re_tmp * ct->f242.re - j_ct_im_tmp * ct->f242.im;
  ct_im = j_ct_re_tmp * ct->f242.im + j_ct_im_tmp * ct->f242.re;
  if (ct_im == 0.0) {
    t1612_re = ct_re / 2.0;
    t1612_im = 0.0;
  } else if (ct_re == 0.0) {
    t1612_re = 0.0;
    t1612_im = ct_im / 2.0;
  } else {
    t1612_re = ct_re / 2.0;
    t1612_im = ct_im / 2.0;
  }

  t1691_re = ct->f601 * ct->f630 * ct->f669;
  k_ct_re_tmp = t1691_re * ct->f233.re;
  k_ct_im_tmp = t1691_re * ct->f233.im;
  ct_re = k_ct_re_tmp * ct->f242.re - k_ct_im_tmp * ct->f242.im;
  ct_im = k_ct_re_tmp * ct->f242.im + k_ct_im_tmp * ct->f242.re;
  if (ct_im == 0.0) {
    t1613_re = ct_re / 2.0;
    t1613_im = 0.0;
  } else if (ct_re == 0.0) {
    t1613_re = 0.0;
    t1613_im = ct_im / 2.0;
  } else {
    t1613_re = ct_re / 2.0;
    t1613_im = ct_im / 2.0;
  }

  t1691_re = ct->f602 * ct->f622 * ct->f671;
  l_ct_re_tmp = t1691_re * ct->f234.re;
  l_ct_im_tmp = t1691_re * ct->f234.im;
  ct_re = l_ct_re_tmp * ct->f243.re - l_ct_im_tmp * ct->f243.im;
  ct_im = l_ct_re_tmp * ct->f243.im + l_ct_im_tmp * ct->f243.re;
  if (ct_im == 0.0) {
    t1614_re = ct_re / 2.0;
    t1614_im = 0.0;
  } else if (ct_re == 0.0) {
    t1614_re = 0.0;
    t1614_im = ct_im / 2.0;
  } else {
    t1614_re = ct_re / 2.0;
    t1614_im = ct_im / 2.0;
  }

  t1691_re = ct->f610 * ct->f630 * ct->f669;
  m_ct_re_tmp = t1691_re * ct->f233.re;
  m_ct_im_tmp = t1691_re * ct->f233.im;
  ct_re = m_ct_re_tmp * ct->f242.re - m_ct_im_tmp * ct->f242.im;
  ct_im = m_ct_re_tmp * ct->f242.im + m_ct_im_tmp * ct->f242.re;
  if (ct_im == 0.0) {
    t1615_re = ct_re / 2.0;
    t1615_im = 0.0;
  } else if (ct_re == 0.0) {
    t1615_re = 0.0;
    t1615_im = ct_im / 2.0;
  } else {
    t1615_re = ct_re / 2.0;
    t1615_im = ct_im / 2.0;
  }

  t1691_re = ct->f611 * ct->f622 * ct->f671;
  n_ct_re_tmp = t1691_re * ct->f234.re;
  n_ct_im_tmp = t1691_re * ct->f234.im;
  ct_re = n_ct_re_tmp * ct->f243.re - n_ct_im_tmp * ct->f243.im;
  ct_im = n_ct_re_tmp * ct->f243.im + n_ct_im_tmp * ct->f243.re;
  if (ct_im == 0.0) {
    t1616_re = ct_re / 2.0;
    t1616_im = 0.0;
  } else if (ct_re == 0.0) {
    t1616_re = 0.0;
    t1616_im = ct_im / 2.0;
  } else {
    t1616_re = ct_re / 2.0;
    t1616_im = ct_im / 2.0;
  }

  t1691_re = ct->f602 * ct->f631 * ct->f671;
  o_ct_re_tmp = t1691_re * ct->f234.re;
  o_ct_im_tmp = t1691_re * ct->f234.im;
  ct_re = o_ct_re_tmp * ct->f243.re - o_ct_im_tmp * ct->f243.im;
  ct_im = o_ct_re_tmp * ct->f243.im + o_ct_im_tmp * ct->f243.re;
  if (ct_im == 0.0) {
    t1617_re = ct_re / 2.0;
    t1617_im = 0.0;
  } else if (ct_re == 0.0) {
    t1617_re = 0.0;
    t1617_im = ct_im / 2.0;
  } else {
    t1617_re = ct_re / 2.0;
    t1617_im = ct_im / 2.0;
  }

  t1691_re = ct->f604 * ct->f623 * ct->f673;
  p_ct_re_tmp = t1691_re * ct->f235.re;
  p_ct_im_tmp = t1691_re * ct->f235.im;
  ct_re = p_ct_re_tmp * ct->f244.re - p_ct_im_tmp * ct->f244.im;
  ct_im = p_ct_re_tmp * ct->f244.im + p_ct_im_tmp * ct->f244.re;
  if (ct_im == 0.0) {
    t1618_re = ct_re / 2.0;
    t1618_im = 0.0;
  } else if (ct_re == 0.0) {
    t1618_re = 0.0;
    t1618_im = ct_im / 2.0;
  } else {
    t1618_re = ct_re / 2.0;
    t1618_im = ct_im / 2.0;
  }

  t1691_re = ct->f611 * ct->f631 * ct->f671;
  q_ct_re_tmp = t1691_re * ct->f234.re;
  q_ct_im_tmp = t1691_re * ct->f234.im;
  ct_re = q_ct_re_tmp * ct->f243.re - q_ct_im_tmp * ct->f243.im;
  ct_im = q_ct_re_tmp * ct->f243.im + q_ct_im_tmp * ct->f243.re;
  if (ct_im == 0.0) {
    t1619_re = ct_re / 2.0;
    t1619_im = 0.0;
  } else if (ct_re == 0.0) {
    t1619_re = 0.0;
    t1619_im = ct_im / 2.0;
  } else {
    t1619_re = ct_re / 2.0;
    t1619_im = ct_im / 2.0;
  }

  t1691_re = ct->f612 * ct->f623 * ct->f673;
  r_ct_re_tmp = t1691_re * ct->f235.re;
  r_ct_im_tmp = t1691_re * ct->f235.im;
  ct_re = r_ct_re_tmp * ct->f244.re - r_ct_im_tmp * ct->f244.im;
  ct_im = r_ct_re_tmp * ct->f244.im + r_ct_im_tmp * ct->f244.re;
  if (ct_im == 0.0) {
    t1620_re = ct_re / 2.0;
    t1620_im = 0.0;
  } else if (ct_re == 0.0) {
    t1620_re = 0.0;
    t1620_im = ct_im / 2.0;
  } else {
    t1620_re = ct_re / 2.0;
    t1620_im = ct_im / 2.0;
  }

  t1691_re = ct->f604 * ct->f632 * ct->f673;
  s_ct_re_tmp = t1691_re * ct->f235.re;
  s_ct_im_tmp = t1691_re * ct->f235.im;
  ct_re = s_ct_re_tmp * ct->f244.re - s_ct_im_tmp * ct->f244.im;
  ct_im = s_ct_re_tmp * ct->f244.im + s_ct_im_tmp * ct->f244.re;
  if (ct_im == 0.0) {
    t1621_re = ct_re / 2.0;
    t1621_im = 0.0;
  } else if (ct_re == 0.0) {
    t1621_re = 0.0;
    t1621_im = ct_im / 2.0;
  } else {
    t1621_re = ct_re / 2.0;
    t1621_im = ct_im / 2.0;
  }

  t1691_re = ct->f605 * ct->f624 * ct->f675;
  t_ct_re_tmp = t1691_re * ct->f236.re;
  t_ct_im_tmp = t1691_re * ct->f236.im;
  ct_re = t_ct_re_tmp * ct->f245.re - t_ct_im_tmp * ct->f245.im;
  ct_im = t_ct_re_tmp * ct->f245.im + t_ct_im_tmp * ct->f245.re;
  if (ct_im == 0.0) {
    t1622_re = ct_re / 2.0;
    t1622_im = 0.0;
  } else if (ct_re == 0.0) {
    t1622_re = 0.0;
    t1622_im = ct_im / 2.0;
  } else {
    t1622_re = ct_re / 2.0;
    t1622_im = ct_im / 2.0;
  }

  t1691_re = ct->f612 * ct->f632 * ct->f673;
  u_ct_re_tmp = t1691_re * ct->f235.re;
  u_ct_im_tmp = t1691_re * ct->f235.im;
  ct_re = u_ct_re_tmp * ct->f244.re - u_ct_im_tmp * ct->f244.im;
  ct_im = u_ct_re_tmp * ct->f244.im + u_ct_im_tmp * ct->f244.re;
  if (ct_im == 0.0) {
    t1623_re = ct_re / 2.0;
    t1623_im = 0.0;
  } else if (ct_re == 0.0) {
    t1623_re = 0.0;
    t1623_im = ct_im / 2.0;
  } else {
    t1623_re = ct_re / 2.0;
    t1623_im = ct_im / 2.0;
  }

  t1691_re = ct->f613 * ct->f624 * ct->f675;
  v_ct_re_tmp = t1691_re * ct->f236.re;
  v_ct_im_tmp = t1691_re * ct->f236.im;
  ct_re = v_ct_re_tmp * ct->f245.re - v_ct_im_tmp * ct->f245.im;
  ct_im = v_ct_re_tmp * ct->f245.im + v_ct_im_tmp * ct->f245.re;
  if (ct_im == 0.0) {
    t1624_re = ct_re / 2.0;
    t1624_im = 0.0;
  } else if (ct_re == 0.0) {
    t1624_re = 0.0;
    t1624_im = ct_im / 2.0;
  } else {
    t1624_re = ct_re / 2.0;
    t1624_im = ct_im / 2.0;
  }

  t1691_re = ct->f605 * ct->f633 * ct->f675;
  w_ct_re_tmp = t1691_re * ct->f236.re;
  w_ct_im_tmp = t1691_re * ct->f236.im;
  ct_re = w_ct_re_tmp * ct->f245.re - w_ct_im_tmp * ct->f245.im;
  ct_im = w_ct_re_tmp * ct->f245.im + w_ct_im_tmp * ct->f245.re;
  if (ct_im == 0.0) {
    t1625_re = ct_re / 2.0;
    t1625_im = 0.0;
  } else if (ct_re == 0.0) {
    t1625_re = 0.0;
    t1625_im = ct_im / 2.0;
  } else {
    t1625_re = ct_re / 2.0;
    t1625_im = ct_im / 2.0;
  }

  t1691_re = ct->f606 * ct->f626 * ct->f677;
  x_ct_re_tmp = t1691_re * ct->f237.re;
  x_ct_im_tmp = t1691_re * ct->f237.im;
  ct_re = x_ct_re_tmp * ct->f246.re - x_ct_im_tmp * ct->f246.im;
  ct_im = x_ct_re_tmp * ct->f246.im + x_ct_im_tmp * ct->f246.re;
  if (ct_im == 0.0) {
    t1626_re = ct_re / 2.0;
    t1626_im = 0.0;
  } else if (ct_re == 0.0) {
    t1626_re = 0.0;
    t1626_im = ct_im / 2.0;
  } else {
    t1626_re = ct_re / 2.0;
    t1626_im = ct_im / 2.0;
  }

  t1691_re = ct->f613 * ct->f633 * ct->f675;
  y_ct_re_tmp = t1691_re * ct->f236.re;
  y_ct_im_tmp = t1691_re * ct->f236.im;
  ct_re = y_ct_re_tmp * ct->f245.re - y_ct_im_tmp * ct->f245.im;
  ct_im = y_ct_re_tmp * ct->f245.im + y_ct_im_tmp * ct->f245.re;
  if (ct_im == 0.0) {
    t1627_re = ct_re / 2.0;
    t1627_im = 0.0;
  } else if (ct_re == 0.0) {
    t1627_re = 0.0;
    t1627_im = ct_im / 2.0;
  } else {
    t1627_re = ct_re / 2.0;
    t1627_im = ct_im / 2.0;
  }

  t1691_re = ct->f615 * ct->f626 * ct->f677;
  ab_ct_re_tmp = t1691_re * ct->f237.re;
  ab_ct_im_tmp = t1691_re * ct->f237.im;
  ct_re = ab_ct_re_tmp * ct->f246.re - ab_ct_im_tmp * ct->f246.im;
  ct_im = ab_ct_re_tmp * ct->f246.im + ab_ct_im_tmp * ct->f246.re;
  if (ct_im == 0.0) {
    t1628_re = ct_re / 2.0;
    t1628_im = 0.0;
  } else if (ct_re == 0.0) {
    t1628_re = 0.0;
    t1628_im = ct_im / 2.0;
  } else {
    t1628_re = ct_re / 2.0;
    t1628_im = ct_im / 2.0;
  }

  t1691_re = ct->f606 * ct->f634 * ct->f677;
  bb_ct_re_tmp = t1691_re * ct->f237.re;
  bb_ct_im_tmp = t1691_re * ct->f237.im;
  ct_re = bb_ct_re_tmp * ct->f246.re - bb_ct_im_tmp * ct->f246.im;
  ct_im = bb_ct_re_tmp * ct->f246.im + bb_ct_im_tmp * ct->f246.re;
  if (ct_im == 0.0) {
    t1629_re = ct_re / 2.0;
    t1629_im = 0.0;
  } else if (ct_re == 0.0) {
    t1629_re = 0.0;
    t1629_im = ct_im / 2.0;
  } else {
    t1629_re = ct_re / 2.0;
    t1629_im = ct_im / 2.0;
  }

  t1691_re = ct->f607 * ct->f627 * ct->f680;
  cb_ct_re_tmp = t1691_re * ct->f238.re;
  cb_ct_im_tmp = t1691_re * ct->f238.im;
  ct_re = cb_ct_re_tmp * ct->f247.re - cb_ct_im_tmp * ct->f247.im;
  ct_im = cb_ct_re_tmp * ct->f247.im + cb_ct_im_tmp * ct->f247.re;
  if (ct_im == 0.0) {
    t1630_re = ct_re / 2.0;
    t1630_im = 0.0;
  } else if (ct_re == 0.0) {
    t1630_re = 0.0;
    t1630_im = ct_im / 2.0;
  } else {
    t1630_re = ct_re / 2.0;
    t1630_im = ct_im / 2.0;
  }

  t1691_re = ct->f615 * ct->f634 * ct->f677;
  db_ct_re_tmp = t1691_re * ct->f237.re;
  db_ct_im_tmp = t1691_re * ct->f237.im;
  ct_re = db_ct_re_tmp * ct->f246.re - db_ct_im_tmp * ct->f246.im;
  ct_im = db_ct_re_tmp * ct->f246.im + db_ct_im_tmp * ct->f246.re;
  if (ct_im == 0.0) {
    t1631_re = ct_re / 2.0;
    t1631_im = 0.0;
  } else if (ct_re == 0.0) {
    t1631_re = 0.0;
    t1631_im = ct_im / 2.0;
  } else {
    t1631_re = ct_re / 2.0;
    t1631_im = ct_im / 2.0;
  }

  t1691_re = ct->f616 * ct->f627 * ct->f680;
  eb_ct_re_tmp = t1691_re * ct->f238.re;
  eb_ct_im_tmp = t1691_re * ct->f238.im;
  ct_re = eb_ct_re_tmp * ct->f247.re - eb_ct_im_tmp * ct->f247.im;
  ct_im = eb_ct_re_tmp * ct->f247.im + eb_ct_im_tmp * ct->f247.re;
  if (ct_im == 0.0) {
    t1632_re = ct_re / 2.0;
    t1632_im = 0.0;
  } else if (ct_re == 0.0) {
    t1632_re = 0.0;
    t1632_im = ct_im / 2.0;
  } else {
    t1632_re = ct_re / 2.0;
    t1632_im = ct_im / 2.0;
  }

  t1691_re = ct->f607 * ct->f635 * ct->f680;
  fb_ct_re_tmp = t1691_re * ct->f238.re;
  fb_ct_im_tmp = t1691_re * ct->f238.im;
  ct_re = fb_ct_re_tmp * ct->f247.re - fb_ct_im_tmp * ct->f247.im;
  ct_im = fb_ct_re_tmp * ct->f247.im + fb_ct_im_tmp * ct->f247.re;
  if (ct_im == 0.0) {
    t1633_re = ct_re / 2.0;
    t1633_im = 0.0;
  } else if (ct_re == 0.0) {
    t1633_re = 0.0;
    t1633_im = ct_im / 2.0;
  } else {
    t1633_re = ct_re / 2.0;
    t1633_im = ct_im / 2.0;
  }

  t1691_re = ct->f616 * ct->f635 * ct->f680;
  gb_ct_re_tmp = t1691_re * ct->f238.re;
  gb_ct_im_tmp = t1691_re * ct->f238.im;
  ct_re = gb_ct_re_tmp * ct->f247.re - gb_ct_im_tmp * ct->f247.im;
  ct_im = gb_ct_re_tmp * ct->f247.im + gb_ct_im_tmp * ct->f247.re;
  if (ct_im == 0.0) {
    t1634_re = ct_re / 2.0;
    t1634_im = 0.0;
  } else if (ct_re == 0.0) {
    t1634_re = 0.0;
    t1634_im = ct_im / 2.0;
  } else {
    t1634_re = ct_re / 2.0;
    t1634_im = ct_im / 2.0;
  }

  t1691_re = ct->f617 * ct->f637 * ct->f682;
  hb_ct_re_tmp = t1691_re * ct->f239.re;
  hb_ct_im_tmp = t1691_re * ct->f239.im;
  ct_re = hb_ct_re_tmp * ct->f248.re - hb_ct_im_tmp * ct->f248.im;
  ct_im = hb_ct_re_tmp * ct->f248.im + hb_ct_im_tmp * ct->f248.re;
  if (ct_im == 0.0) {
    t1635_re = ct_re / 2.0;
    t1635_im = 0.0;
  } else if (ct_re == 0.0) {
    t1635_re = 0.0;
    t1635_im = ct_im / 2.0;
  } else {
    t1635_re = ct_re / 2.0;
    t1635_im = ct_im / 2.0;
  }

  t1691_re = ct->f618 * ct->f637 * ct->f682;
  ib_ct_re_tmp = t1691_re * ct->f239.re;
  ib_ct_im_tmp = t1691_re * ct->f239.im;
  ct_re = ib_ct_re_tmp * ct->f248.re - ib_ct_im_tmp * ct->f248.im;
  ct_im = ib_ct_re_tmp * ct->f248.im + ib_ct_im_tmp * ct->f248.re;
  if (ct_im == 0.0) {
    t1636_re = ct_re / 2.0;
    t1636_im = 0.0;
  } else if (ct_re == 0.0) {
    t1636_re = 0.0;
    t1636_im = ct_im / 2.0;
  } else {
    t1636_re = ct_re / 2.0;
    t1636_im = ct_im / 2.0;
  }

  t1691_re = ct->f617 * ct->f638 * ct->f682;
  jb_ct_re_tmp = t1691_re * ct->f239.re;
  jb_ct_im_tmp = t1691_re * ct->f239.im;
  ct_re = jb_ct_re_tmp * ct->f248.re - jb_ct_im_tmp * ct->f248.im;
  ct_im = jb_ct_re_tmp * ct->f248.im + jb_ct_im_tmp * ct->f248.re;
  if (ct_im == 0.0) {
    t1637_re = ct_re / 2.0;
    t1637_im = 0.0;
  } else if (ct_re == 0.0) {
    t1637_re = 0.0;
    t1637_im = ct_im / 2.0;
  } else {
    t1637_re = ct_re / 2.0;
    t1637_im = ct_im / 2.0;
  }

  t1691_re = ct->f618 * ct->f638 * ct->f682;
  kb_ct_re_tmp = t1691_re * ct->f239.re;
  kb_ct_im_tmp = t1691_re * ct->f239.im;
  ct_re = kb_ct_re_tmp * ct->f248.re - kb_ct_im_tmp * ct->f248.im;
  ct_im = kb_ct_re_tmp * ct->f248.im + kb_ct_im_tmp * ct->f248.re;
  if (ct_im == 0.0) {
    t1638_re = ct_re / 2.0;
    t1638_im = 0.0;
  } else if (ct_re == 0.0) {
    t1638_re = 0.0;
    t1638_im = ct_im / 2.0;
  } else {
    t1638_re = ct_re / 2.0;
    t1638_im = ct_im / 2.0;
  }

  ct_re = ct_re_tmp * ct->f249.re - ct_im_tmp * ct->f249.im;
  ct_im = ct_re_tmp * ct->f249.im + ct_im_tmp * ct->f249.re;
  if (ct_im == 0.0) {
    t1639_re = ct_re / 2.0;
    t1639_im = 0.0;
  } else if (ct_re == 0.0) {
    t1639_re = 0.0;
    t1639_im = ct_im / 2.0;
  } else {
    t1639_re = ct_re / 2.0;
    t1639_im = ct_im / 2.0;
  }

  ct_re = ct_re_tmp * ct->f250.re - ct_im_tmp * ct->f250.im;
  ct_im = ct_re_tmp * ct->f250.im + ct_im_tmp * ct->f250.re;
  if (ct_im == 0.0) {
    t1640_re = ct_re / 2.0;
    t1640_im = 0.0;
  } else if (ct_re == 0.0) {
    t1640_re = 0.0;
    t1640_im = ct_im / 2.0;
  } else {
    t1640_re = ct_re / 2.0;
    t1640_im = ct_im / 2.0;
  }

  ct_re = ct_re_tmp * ct->f251.re - ct_im_tmp * ct->f251.im;
  ct_im = ct_re_tmp * ct->f251.im + ct_im_tmp * ct->f251.re;
  if (ct_im == 0.0) {
    t1641_re = ct_re / 2.0;
    t1641_im = 0.0;
  } else if (ct_re == 0.0) {
    t1641_re = 0.0;
    t1641_im = ct_im / 2.0;
  } else {
    t1641_re = ct_re / 2.0;
    t1641_im = ct_im / 2.0;
  }

  ct_re = b_ct_re_tmp * ct->f249.re - b_ct_im_tmp * ct->f249.im;
  ct_im = b_ct_re_tmp * ct->f249.im + b_ct_im_tmp * ct->f249.re;
  if (ct_im == 0.0) {
    t1642_re = ct_re / 2.0;
    t1642_im = 0.0;
  } else if (ct_re == 0.0) {
    t1642_re = 0.0;
    t1642_im = ct_im / 2.0;
  } else {
    t1642_re = ct_re / 2.0;
    t1642_im = ct_im / 2.0;
  }

  ct_re = b_ct_re_tmp * ct->f250.re - b_ct_im_tmp * ct->f250.im;
  ct_im = b_ct_re_tmp * ct->f250.im + b_ct_im_tmp * ct->f250.re;
  if (ct_im == 0.0) {
    t1643_re = ct_re / 2.0;
    t1643_im = 0.0;
  } else if (ct_re == 0.0) {
    t1643_re = 0.0;
    t1643_im = ct_im / 2.0;
  } else {
    t1643_re = ct_re / 2.0;
    t1643_im = ct_im / 2.0;
  }

  ct_re = c_ct_re_tmp * ct->f249.re - c_ct_im_tmp * ct->f249.im;
  ct_im = c_ct_re_tmp * ct->f249.im + c_ct_im_tmp * ct->f249.re;
  if (ct_im == 0.0) {
    t1644_re = ct_re / 2.0;
    t1644_im = 0.0;
  } else if (ct_re == 0.0) {
    t1644_re = 0.0;
    t1644_im = ct_im / 2.0;
  } else {
    t1644_re = ct_re / 2.0;
    t1644_im = ct_im / 2.0;
  }

  ct_re = b_ct_re_tmp * ct->f251.re - b_ct_im_tmp * ct->f251.im;
  ct_im = b_ct_re_tmp * ct->f251.im + b_ct_im_tmp * ct->f251.re;
  if (ct_im == 0.0) {
    t1645_re = ct_re / 2.0;
    t1645_im = 0.0;
  } else if (ct_re == 0.0) {
    t1645_re = 0.0;
    t1645_im = ct_im / 2.0;
  } else {
    t1645_re = ct_re / 2.0;
    t1645_im = ct_im / 2.0;
  }

  ct_re = c_ct_re_tmp * ct->f250.re - c_ct_im_tmp * ct->f250.im;
  ct_im = c_ct_re_tmp * ct->f250.im + c_ct_im_tmp * ct->f250.re;
  if (ct_im == 0.0) {
    t1646_re = ct_re / 2.0;
    t1646_im = 0.0;
  } else if (ct_re == 0.0) {
    t1646_re = 0.0;
    t1646_im = ct_im / 2.0;
  } else {
    t1646_re = ct_re / 2.0;
    t1646_im = ct_im / 2.0;
  }

  ct_re = c_ct_re_tmp * ct->f251.re - c_ct_im_tmp * ct->f251.im;
  ct_im = c_ct_re_tmp * ct->f251.im + c_ct_im_tmp * ct->f251.re;
  if (ct_im == 0.0) {
    t1647_re = ct_re / 2.0;
    t1647_im = 0.0;
  } else if (ct_re == 0.0) {
    t1647_re = 0.0;
    t1647_im = ct_im / 2.0;
  } else {
    t1647_re = ct_re / 2.0;
    t1647_im = ct_im / 2.0;
  }

  ct_re = d_ct_re_tmp * ct->f252.re - d_ct_im_tmp * ct->f252.im;
  ct_im = d_ct_re_tmp * ct->f252.im + d_ct_im_tmp * ct->f252.re;
  if (ct_im == 0.0) {
    t1648_re = ct_re / 2.0;
    t1648_im = 0.0;
  } else if (ct_re == 0.0) {
    t1648_re = 0.0;
    t1648_im = ct_im / 2.0;
  } else {
    t1648_re = ct_re / 2.0;
    t1648_im = ct_im / 2.0;
  }

  ct_re = d_ct_re_tmp * ct->f253.re - d_ct_im_tmp * ct->f253.im;
  ct_im = d_ct_re_tmp * ct->f253.im + d_ct_im_tmp * ct->f253.re;
  if (ct_im == 0.0) {
    t1649_re = ct_re / 2.0;
    t1649_im = 0.0;
  } else if (ct_re == 0.0) {
    t1649_re = 0.0;
    t1649_im = ct_im / 2.0;
  } else {
    t1649_re = ct_re / 2.0;
    t1649_im = ct_im / 2.0;
  }

  ct_re = e_ct_re_tmp * ct->f249.re - e_ct_im_tmp * ct->f249.im;
  ct_im = e_ct_re_tmp * ct->f249.im + e_ct_im_tmp * ct->f249.re;
  if (ct_im == 0.0) {
    t1650_re = ct_re / 2.0;
    t1650_im = 0.0;
  } else if (ct_re == 0.0) {
    t1650_re = 0.0;
    t1650_im = ct_im / 2.0;
  } else {
    t1650_re = ct_re / 2.0;
    t1650_im = ct_im / 2.0;
  }

  ct_re = d_ct_re_tmp * ct->f254.re - d_ct_im_tmp * ct->f254.im;
  ct_im = d_ct_re_tmp * ct->f254.im + d_ct_im_tmp * ct->f254.re;
  if (ct_im == 0.0) {
    t1651_re = ct_re / 2.0;
    t1651_im = 0.0;
  } else if (ct_re == 0.0) {
    t1651_re = 0.0;
    t1651_im = ct_im / 2.0;
  } else {
    t1651_re = ct_re / 2.0;
    t1651_im = ct_im / 2.0;
  }

  ct_re = e_ct_re_tmp * ct->f250.re - e_ct_im_tmp * ct->f250.im;
  ct_im = e_ct_re_tmp * ct->f250.im + e_ct_im_tmp * ct->f250.re;
  if (ct_im == 0.0) {
    t1652_re = ct_re / 2.0;
    t1652_im = 0.0;
  } else if (ct_re == 0.0) {
    t1652_re = 0.0;
    t1652_im = ct_im / 2.0;
  } else {
    t1652_re = ct_re / 2.0;
    t1652_im = ct_im / 2.0;
  }

  ct_re = e_ct_re_tmp * ct->f251.re - e_ct_im_tmp * ct->f251.im;
  ct_im = e_ct_re_tmp * ct->f251.im + e_ct_im_tmp * ct->f251.re;
  if (ct_im == 0.0) {
    t1653_re = ct_re / 2.0;
    t1653_im = 0.0;
  } else if (ct_re == 0.0) {
    t1653_re = 0.0;
    t1653_im = ct_im / 2.0;
  } else {
    t1653_re = ct_re / 2.0;
    t1653_im = ct_im / 2.0;
  }

  ct_re = f_ct_re_tmp * ct->f252.re - f_ct_im_tmp * ct->f252.im;
  ct_im = f_ct_re_tmp * ct->f252.im + f_ct_im_tmp * ct->f252.re;
  if (ct_im == 0.0) {
    t1654_re = ct_re / 2.0;
    t1654_im = 0.0;
  } else if (ct_re == 0.0) {
    t1654_re = 0.0;
    t1654_im = ct_im / 2.0;
  } else {
    t1654_re = ct_re / 2.0;
    t1654_im = ct_im / 2.0;
  }

  ct_re = f_ct_re_tmp * ct->f253.re - f_ct_im_tmp * ct->f253.im;
  ct_im = f_ct_re_tmp * ct->f253.im + f_ct_im_tmp * ct->f253.re;
  if (ct_im == 0.0) {
    t1655_re = ct_re / 2.0;
    t1655_im = 0.0;
  } else if (ct_re == 0.0) {
    t1655_re = 0.0;
    t1655_im = ct_im / 2.0;
  } else {
    t1655_re = ct_re / 2.0;
    t1655_im = ct_im / 2.0;
  }

  ct_re = g_ct_re_tmp * ct->f252.re - g_ct_im_tmp * ct->f252.im;
  ct_im = g_ct_re_tmp * ct->f252.im + g_ct_im_tmp * ct->f252.re;
  if (ct_im == 0.0) {
    t1656_re = ct_re / 2.0;
    t1656_im = 0.0;
  } else if (ct_re == 0.0) {
    t1656_re = 0.0;
    t1656_im = ct_im / 2.0;
  } else {
    t1656_re = ct_re / 2.0;
    t1656_im = ct_im / 2.0;
  }

  ct_re = f_ct_re_tmp * ct->f254.re - f_ct_im_tmp * ct->f254.im;
  ct_im = f_ct_re_tmp * ct->f254.im + f_ct_im_tmp * ct->f254.re;
  if (ct_im == 0.0) {
    t1657_re = ct_re / 2.0;
    t1657_im = 0.0;
  } else if (ct_re == 0.0) {
    t1657_re = 0.0;
    t1657_im = ct_im / 2.0;
  } else {
    t1657_re = ct_re / 2.0;
    t1657_im = ct_im / 2.0;
  }

  ct_re = g_ct_re_tmp * ct->f253.re - g_ct_im_tmp * ct->f253.im;
  ct_im = g_ct_re_tmp * ct->f253.im + g_ct_im_tmp * ct->f253.re;
  if (ct_im == 0.0) {
    t1658_re = ct_re / 2.0;
    t1658_im = 0.0;
  } else if (ct_re == 0.0) {
    t1658_re = 0.0;
    t1658_im = ct_im / 2.0;
  } else {
    t1658_re = ct_re / 2.0;
    t1658_im = ct_im / 2.0;
  }

  ct_re = g_ct_re_tmp * ct->f254.re - g_ct_im_tmp * ct->f254.im;
  ct_im = g_ct_re_tmp * ct->f254.im + g_ct_im_tmp * ct->f254.re;
  if (ct_im == 0.0) {
    t1659_re = ct_re / 2.0;
    t1659_im = 0.0;
  } else if (ct_re == 0.0) {
    t1659_re = 0.0;
    t1659_im = ct_im / 2.0;
  } else {
    t1659_re = ct_re / 2.0;
    t1659_im = ct_im / 2.0;
  }

  ct_re = h_ct_re_tmp * ct->f255.re - h_ct_im_tmp * ct->f255.im;
  ct_im = h_ct_re_tmp * ct->f255.im + h_ct_im_tmp * ct->f255.re;
  if (ct_im == 0.0) {
    t1660_re = ct_re / 2.0;
    t1660_im = 0.0;
  } else if (ct_re == 0.0) {
    t1660_re = 0.0;
    t1660_im = ct_im / 2.0;
  } else {
    t1660_re = ct_re / 2.0;
    t1660_im = ct_im / 2.0;
  }

  ct_re = h_ct_re_tmp * ct->f256.re - h_ct_im_tmp * ct->f256.im;
  ct_im = h_ct_re_tmp * ct->f256.im + h_ct_im_tmp * ct->f256.re;
  if (ct_im == 0.0) {
    t1661_re = ct_re / 2.0;
    t1661_im = 0.0;
  } else if (ct_re == 0.0) {
    t1661_re = 0.0;
    t1661_im = ct_im / 2.0;
  } else {
    t1661_re = ct_re / 2.0;
    t1661_im = ct_im / 2.0;
  }

  ct_re = i_ct_re_tmp * ct->f252.re - i_ct_im_tmp * ct->f252.im;
  ct_im = i_ct_re_tmp * ct->f252.im + i_ct_im_tmp * ct->f252.re;
  if (ct_im == 0.0) {
    t1662_re = ct_re / 2.0;
    t1662_im = 0.0;
  } else if (ct_re == 0.0) {
    t1662_re = 0.0;
    t1662_im = ct_im / 2.0;
  } else {
    t1662_re = ct_re / 2.0;
    t1662_im = ct_im / 2.0;
  }

  ct_re = h_ct_re_tmp * ct->f257.re - h_ct_im_tmp * ct->f257.im;
  ct_im = h_ct_re_tmp * ct->f257.im + h_ct_im_tmp * ct->f257.re;
  if (ct_im == 0.0) {
    t1663_re = ct_re / 2.0;
    t1663_im = 0.0;
  } else if (ct_re == 0.0) {
    t1663_re = 0.0;
    t1663_im = ct_im / 2.0;
  } else {
    t1663_re = ct_re / 2.0;
    t1663_im = ct_im / 2.0;
  }

  ct_re = i_ct_re_tmp * ct->f253.re - i_ct_im_tmp * ct->f253.im;
  ct_im = i_ct_re_tmp * ct->f253.im + i_ct_im_tmp * ct->f253.re;
  if (ct_im == 0.0) {
    t1664_re = ct_re / 2.0;
    t1664_im = 0.0;
  } else if (ct_re == 0.0) {
    t1664_re = 0.0;
    t1664_im = ct_im / 2.0;
  } else {
    t1664_re = ct_re / 2.0;
    t1664_im = ct_im / 2.0;
  }

  ct_re = i_ct_re_tmp * ct->f254.re - i_ct_im_tmp * ct->f254.im;
  ct_im = i_ct_re_tmp * ct->f254.im + i_ct_im_tmp * ct->f254.re;
  if (ct_im == 0.0) {
    t1665_re = ct_re / 2.0;
    t1665_im = 0.0;
  } else if (ct_re == 0.0) {
    t1665_re = 0.0;
    t1665_im = ct_im / 2.0;
  } else {
    t1665_re = ct_re / 2.0;
    t1665_im = ct_im / 2.0;
  }

  ct_re = j_ct_re_tmp * ct->f255.re - j_ct_im_tmp * ct->f255.im;
  ct_im = j_ct_re_tmp * ct->f255.im + j_ct_im_tmp * ct->f255.re;
  if (ct_im == 0.0) {
    t1666_re = ct_re / 2.0;
    t1666_im = 0.0;
  } else if (ct_re == 0.0) {
    t1666_re = 0.0;
    t1666_im = ct_im / 2.0;
  } else {
    t1666_re = ct_re / 2.0;
    t1666_im = ct_im / 2.0;
  }

  ct_re = j_ct_re_tmp * ct->f256.re - j_ct_im_tmp * ct->f256.im;
  ct_im = j_ct_re_tmp * ct->f256.im + j_ct_im_tmp * ct->f256.re;
  if (ct_im == 0.0) {
    t1667_re = ct_re / 2.0;
    t1667_im = 0.0;
  } else if (ct_re == 0.0) {
    t1667_re = 0.0;
    t1667_im = ct_im / 2.0;
  } else {
    t1667_re = ct_re / 2.0;
    t1667_im = ct_im / 2.0;
  }

  ct_re = k_ct_re_tmp * ct->f255.re - k_ct_im_tmp * ct->f255.im;
  ct_im = k_ct_re_tmp * ct->f255.im + k_ct_im_tmp * ct->f255.re;
  if (ct_im == 0.0) {
    t1668_re = ct_re / 2.0;
    t1668_im = 0.0;
  } else if (ct_re == 0.0) {
    t1668_re = 0.0;
    t1668_im = ct_im / 2.0;
  } else {
    t1668_re = ct_re / 2.0;
    t1668_im = ct_im / 2.0;
  }

  ct_re = j_ct_re_tmp * ct->f257.re - j_ct_im_tmp * ct->f257.im;
  ct_im = j_ct_re_tmp * ct->f257.im + j_ct_im_tmp * ct->f257.re;
  if (ct_im == 0.0) {
    t1669_re = ct_re / 2.0;
    t1669_im = 0.0;
  } else if (ct_re == 0.0) {
    t1669_re = 0.0;
    t1669_im = ct_im / 2.0;
  } else {
    t1669_re = ct_re / 2.0;
    t1669_im = ct_im / 2.0;
  }

  ct_re = k_ct_re_tmp * ct->f256.re - k_ct_im_tmp * ct->f256.im;
  ct_im = k_ct_re_tmp * ct->f256.im + k_ct_im_tmp * ct->f256.re;
  if (ct_im == 0.0) {
    t1670_re = ct_re / 2.0;
    t1670_im = 0.0;
  } else if (ct_re == 0.0) {
    t1670_re = 0.0;
    t1670_im = ct_im / 2.0;
  } else {
    t1670_re = ct_re / 2.0;
    t1670_im = ct_im / 2.0;
  }

  ct_re = k_ct_re_tmp * ct->f257.re - k_ct_im_tmp * ct->f257.im;
  ct_im = k_ct_re_tmp * ct->f257.im + k_ct_im_tmp * ct->f257.re;
  if (ct_im == 0.0) {
    t1671_re = ct_re / 2.0;
    t1671_im = 0.0;
  } else if (ct_re == 0.0) {
    t1671_re = 0.0;
    t1671_im = ct_im / 2.0;
  } else {
    t1671_re = ct_re / 2.0;
    t1671_im = ct_im / 2.0;
  }

  ct_re = l_ct_re_tmp * ct->f258.re - l_ct_im_tmp * ct->f258.im;
  ct_im = l_ct_re_tmp * ct->f258.im + l_ct_im_tmp * ct->f258.re;
  if (ct_im == 0.0) {
    t1672_re = ct_re / 2.0;
    t1672_im = 0.0;
  } else if (ct_re == 0.0) {
    t1672_re = 0.0;
    t1672_im = ct_im / 2.0;
  } else {
    t1672_re = ct_re / 2.0;
    t1672_im = ct_im / 2.0;
  }

  ct_re = l_ct_re_tmp * ct->f259.re - l_ct_im_tmp * ct->f259.im;
  ct_im = l_ct_re_tmp * ct->f259.im + l_ct_im_tmp * ct->f259.re;
  if (ct_im == 0.0) {
    t1673_re = ct_re / 2.0;
    t1673_im = 0.0;
  } else if (ct_re == 0.0) {
    t1673_re = 0.0;
    t1673_im = ct_im / 2.0;
  } else {
    t1673_re = ct_re / 2.0;
    t1673_im = ct_im / 2.0;
  }

  ct_re = m_ct_re_tmp * ct->f255.re - m_ct_im_tmp * ct->f255.im;
  ct_im = m_ct_re_tmp * ct->f255.im + m_ct_im_tmp * ct->f255.re;
  if (ct_im == 0.0) {
    t1674_re = ct_re / 2.0;
    t1674_im = 0.0;
  } else if (ct_re == 0.0) {
    t1674_re = 0.0;
    t1674_im = ct_im / 2.0;
  } else {
    t1674_re = ct_re / 2.0;
    t1674_im = ct_im / 2.0;
  }

  ct_re = l_ct_re_tmp * ct->f260.re - l_ct_im_tmp * ct->f260.im;
  ct_im = l_ct_re_tmp * ct->f260.im + l_ct_im_tmp * ct->f260.re;
  if (ct_im == 0.0) {
    t1675_re = ct_re / 2.0;
    t1675_im = 0.0;
  } else if (ct_re == 0.0) {
    t1675_re = 0.0;
    t1675_im = ct_im / 2.0;
  } else {
    t1675_re = ct_re / 2.0;
    t1675_im = ct_im / 2.0;
  }

  ct_re = m_ct_re_tmp * ct->f256.re - m_ct_im_tmp * ct->f256.im;
  ct_im = m_ct_re_tmp * ct->f256.im + m_ct_im_tmp * ct->f256.re;
  if (ct_im == 0.0) {
    t1676_re = ct_re / 2.0;
    t1676_im = 0.0;
  } else if (ct_re == 0.0) {
    t1676_re = 0.0;
    t1676_im = ct_im / 2.0;
  } else {
    t1676_re = ct_re / 2.0;
    t1676_im = ct_im / 2.0;
  }

  ct_re = m_ct_re_tmp * ct->f257.re - m_ct_im_tmp * ct->f257.im;
  ct_im = m_ct_re_tmp * ct->f257.im + m_ct_im_tmp * ct->f257.re;
  if (ct_im == 0.0) {
    t1677_re = ct_re / 2.0;
    t1677_im = 0.0;
  } else if (ct_re == 0.0) {
    t1677_re = 0.0;
    t1677_im = ct_im / 2.0;
  } else {
    t1677_re = ct_re / 2.0;
    t1677_im = ct_im / 2.0;
  }

  ct_re = n_ct_re_tmp * ct->f258.re - n_ct_im_tmp * ct->f258.im;
  ct_im = n_ct_re_tmp * ct->f258.im + n_ct_im_tmp * ct->f258.re;
  if (ct_im == 0.0) {
    t1678_re = ct_re / 2.0;
    t1678_im = 0.0;
  } else if (ct_re == 0.0) {
    t1678_re = 0.0;
    t1678_im = ct_im / 2.0;
  } else {
    t1678_re = ct_re / 2.0;
    t1678_im = ct_im / 2.0;
  }

  ct_re = n_ct_re_tmp * ct->f259.re - n_ct_im_tmp * ct->f259.im;
  ct_im = n_ct_re_tmp * ct->f259.im + n_ct_im_tmp * ct->f259.re;
  if (ct_im == 0.0) {
    t1679_re = ct_re / 2.0;
    t1679_im = 0.0;
  } else if (ct_re == 0.0) {
    t1679_re = 0.0;
    t1679_im = ct_im / 2.0;
  } else {
    t1679_re = ct_re / 2.0;
    t1679_im = ct_im / 2.0;
  }

  ct_re = o_ct_re_tmp * ct->f258.re - o_ct_im_tmp * ct->f258.im;
  ct_im = o_ct_re_tmp * ct->f258.im + o_ct_im_tmp * ct->f258.re;
  if (ct_im == 0.0) {
    t1680_re = ct_re / 2.0;
    t1680_im = 0.0;
  } else if (ct_re == 0.0) {
    t1680_re = 0.0;
    t1680_im = ct_im / 2.0;
  } else {
    t1680_re = ct_re / 2.0;
    t1680_im = ct_im / 2.0;
  }

  ct_re = n_ct_re_tmp * ct->f260.re - n_ct_im_tmp * ct->f260.im;
  ct_im = n_ct_re_tmp * ct->f260.im + n_ct_im_tmp * ct->f260.re;
  if (ct_im == 0.0) {
    t1681_re = ct_re / 2.0;
    t1681_im = 0.0;
  } else if (ct_re == 0.0) {
    t1681_re = 0.0;
    t1681_im = ct_im / 2.0;
  } else {
    t1681_re = ct_re / 2.0;
    t1681_im = ct_im / 2.0;
  }

  ct_re = o_ct_re_tmp * ct->f259.re - o_ct_im_tmp * ct->f259.im;
  ct_im = o_ct_re_tmp * ct->f259.im + o_ct_im_tmp * ct->f259.re;
  if (ct_im == 0.0) {
    t1682_re = ct_re / 2.0;
    t1682_im = 0.0;
  } else if (ct_re == 0.0) {
    t1682_re = 0.0;
    t1682_im = ct_im / 2.0;
  } else {
    t1682_re = ct_re / 2.0;
    t1682_im = ct_im / 2.0;
  }

  ct_re = o_ct_re_tmp * ct->f260.re - o_ct_im_tmp * ct->f260.im;
  ct_im = o_ct_re_tmp * ct->f260.im + o_ct_im_tmp * ct->f260.re;
  if (ct_im == 0.0) {
    t1683_re = ct_re / 2.0;
    t1683_im = 0.0;
  } else if (ct_re == 0.0) {
    t1683_re = 0.0;
    t1683_im = ct_im / 2.0;
  } else {
    t1683_re = ct_re / 2.0;
    t1683_im = ct_im / 2.0;
  }

  ct_re = p_ct_re_tmp * ct->f262.re - p_ct_im_tmp * ct->f262.im;
  ct_im = p_ct_re_tmp * ct->f262.im + p_ct_im_tmp * ct->f262.re;
  if (ct_im == 0.0) {
    t1684_re = ct_re / 2.0;
    t1684_im = 0.0;
  } else if (ct_re == 0.0) {
    t1684_re = 0.0;
    t1684_im = ct_im / 2.0;
  } else {
    t1684_re = ct_re / 2.0;
    t1684_im = ct_im / 2.0;
  }

  ct_re = p_ct_re_tmp * ct->f263.re - p_ct_im_tmp * ct->f263.im;
  ct_im = p_ct_re_tmp * ct->f263.im + p_ct_im_tmp * ct->f263.re;
  if (ct_im == 0.0) {
    t1685_re = ct_re / 2.0;
    t1685_im = 0.0;
  } else if (ct_re == 0.0) {
    t1685_re = 0.0;
    t1685_im = ct_im / 2.0;
  } else {
    t1685_re = ct_re / 2.0;
    t1685_im = ct_im / 2.0;
  }

  ct_re = q_ct_re_tmp * ct->f258.re - q_ct_im_tmp * ct->f258.im;
  ct_im = q_ct_re_tmp * ct->f258.im + q_ct_im_tmp * ct->f258.re;
  if (ct_im == 0.0) {
    t1686_re = ct_re / 2.0;
    t1686_im = 0.0;
  } else if (ct_re == 0.0) {
    t1686_re = 0.0;
    t1686_im = ct_im / 2.0;
  } else {
    t1686_re = ct_re / 2.0;
    t1686_im = ct_im / 2.0;
  }

  ct_re = p_ct_re_tmp * ct->f264.re - p_ct_im_tmp * ct->f264.im;
  ct_im = p_ct_re_tmp * ct->f264.im + p_ct_im_tmp * ct->f264.re;
  if (ct_im == 0.0) {
    t1687_re = ct_re / 2.0;
    t1687_im = 0.0;
  } else if (ct_re == 0.0) {
    t1687_re = 0.0;
    t1687_im = ct_im / 2.0;
  } else {
    t1687_re = ct_re / 2.0;
    t1687_im = ct_im / 2.0;
  }

  ct_re = q_ct_re_tmp * ct->f259.re - q_ct_im_tmp * ct->f259.im;
  ct_im = q_ct_re_tmp * ct->f259.im + q_ct_im_tmp * ct->f259.re;
  if (ct_im == 0.0) {
    t1688_re = ct_re / 2.0;
    t1688_im = 0.0;
  } else if (ct_re == 0.0) {
    t1688_re = 0.0;
    t1688_im = ct_im / 2.0;
  } else {
    t1688_re = ct_re / 2.0;
    t1688_im = ct_im / 2.0;
  }

  ct_re = q_ct_re_tmp * ct->f260.re - q_ct_im_tmp * ct->f260.im;
  ct_im = q_ct_re_tmp * ct->f260.im + q_ct_im_tmp * ct->f260.re;
  if (ct_im == 0.0) {
    t1689_re = ct_re / 2.0;
    t1689_im = 0.0;
  } else if (ct_re == 0.0) {
    t1689_re = 0.0;
    t1689_im = ct_im / 2.0;
  } else {
    t1689_re = ct_re / 2.0;
    t1689_im = ct_im / 2.0;
  }

  ct_re = r_ct_re_tmp * ct->f262.re - r_ct_im_tmp * ct->f262.im;
  ct_im = r_ct_re_tmp * ct->f262.im + r_ct_im_tmp * ct->f262.re;
  if (ct_im == 0.0) {
    t1690_re = ct_re / 2.0;
    t1690_im = 0.0;
  } else if (ct_re == 0.0) {
    t1690_re = 0.0;
    t1690_im = ct_im / 2.0;
  } else {
    t1690_re = ct_re / 2.0;
    t1690_im = ct_im / 2.0;
  }

  ct_re = r_ct_re_tmp * ct->f263.re - r_ct_im_tmp * ct->f263.im;
  ct_im = r_ct_re_tmp * ct->f263.im + r_ct_im_tmp * ct->f263.re;
  if (ct_im == 0.0) {
    t1691_re = ct_re / 2.0;
    t1691_im = 0.0;
  } else if (ct_re == 0.0) {
    t1691_re = 0.0;
    t1691_im = ct_im / 2.0;
  } else {
    t1691_re = ct_re / 2.0;
    t1691_im = ct_im / 2.0;
  }

  ct_re = s_ct_re_tmp * ct->f262.re - s_ct_im_tmp * ct->f262.im;
  ct_im = s_ct_re_tmp * ct->f262.im + s_ct_im_tmp * ct->f262.re;
  if (ct_im == 0.0) {
    t1692_re = ct_re / 2.0;
    t1692_im = 0.0;
  } else if (ct_re == 0.0) {
    t1692_re = 0.0;
    t1692_im = ct_im / 2.0;
  } else {
    t1692_re = ct_re / 2.0;
    t1692_im = ct_im / 2.0;
  }

  ct_re = r_ct_re_tmp * ct->f264.re - r_ct_im_tmp * ct->f264.im;
  ct_im = r_ct_re_tmp * ct->f264.im + r_ct_im_tmp * ct->f264.re;
  if (ct_im == 0.0) {
    t1693_re = ct_re / 2.0;
    t1693_im = 0.0;
  } else if (ct_re == 0.0) {
    t1693_re = 0.0;
    t1693_im = ct_im / 2.0;
  } else {
    t1693_re = ct_re / 2.0;
    t1693_im = ct_im / 2.0;
  }

  ct_re = s_ct_re_tmp * ct->f263.re - s_ct_im_tmp * ct->f263.im;
  ct_im = s_ct_re_tmp * ct->f263.im + s_ct_im_tmp * ct->f263.re;
  if (ct_im == 0.0) {
    t1694_re = ct_re / 2.0;
    t1694_im = 0.0;
  } else if (ct_re == 0.0) {
    t1694_re = 0.0;
    t1694_im = ct_im / 2.0;
  } else {
    t1694_re = ct_re / 2.0;
    t1694_im = ct_im / 2.0;
  }

  ct_re = s_ct_re_tmp * ct->f264.re - s_ct_im_tmp * ct->f264.im;
  ct_im = s_ct_re_tmp * ct->f264.im + s_ct_im_tmp * ct->f264.re;
  if (ct_im == 0.0) {
    t1695_re = ct_re / 2.0;
    t1695_im = 0.0;
  } else if (ct_re == 0.0) {
    t1695_re = 0.0;
    t1695_im = ct_im / 2.0;
  } else {
    t1695_re = ct_re / 2.0;
    t1695_im = ct_im / 2.0;
  }

  ct_re = t_ct_re_tmp * ct->f265.re - t_ct_im_tmp * ct->f265.im;
  ct_im = t_ct_re_tmp * ct->f265.im + t_ct_im_tmp * ct->f265.re;
  if (ct_im == 0.0) {
    t1696_re = ct_re / 2.0;
    t1696_im = 0.0;
  } else if (ct_re == 0.0) {
    t1696_re = 0.0;
    t1696_im = ct_im / 2.0;
  } else {
    t1696_re = ct_re / 2.0;
    t1696_im = ct_im / 2.0;
  }

  ct_re = t_ct_re_tmp * ct->f266.re - t_ct_im_tmp * ct->f266.im;
  ct_im = t_ct_re_tmp * ct->f266.im + t_ct_im_tmp * ct->f266.re;
  if (ct_im == 0.0) {
    t1697_re = ct_re / 2.0;
    t1697_im = 0.0;
  } else if (ct_re == 0.0) {
    t1697_re = 0.0;
    t1697_im = ct_im / 2.0;
  } else {
    t1697_re = ct_re / 2.0;
    t1697_im = ct_im / 2.0;
  }

  ct_re = u_ct_re_tmp * ct->f262.re - u_ct_im_tmp * ct->f262.im;
  ct_im = u_ct_re_tmp * ct->f262.im + u_ct_im_tmp * ct->f262.re;
  if (ct_im == 0.0) {
    t1698_re = ct_re / 2.0;
    t1698_im = 0.0;
  } else if (ct_re == 0.0) {
    t1698_re = 0.0;
    t1698_im = ct_im / 2.0;
  } else {
    t1698_re = ct_re / 2.0;
    t1698_im = ct_im / 2.0;
  }

  ct_re = t_ct_re_tmp * ct->f267.re - t_ct_im_tmp * ct->f267.im;
  ct_im = t_ct_re_tmp * ct->f267.im + t_ct_im_tmp * ct->f267.re;
  if (ct_im == 0.0) {
    t1699_re = ct_re / 2.0;
    t1699_im = 0.0;
  } else if (ct_re == 0.0) {
    t1699_re = 0.0;
    t1699_im = ct_im / 2.0;
  } else {
    t1699_re = ct_re / 2.0;
    t1699_im = ct_im / 2.0;
  }

  ct_re = u_ct_re_tmp * ct->f263.re - u_ct_im_tmp * ct->f263.im;
  ct_im = u_ct_re_tmp * ct->f263.im + u_ct_im_tmp * ct->f263.re;
  if (ct_im == 0.0) {
    t1700_re = ct_re / 2.0;
    t1700_im = 0.0;
  } else if (ct_re == 0.0) {
    t1700_re = 0.0;
    t1700_im = ct_im / 2.0;
  } else {
    t1700_re = ct_re / 2.0;
    t1700_im = ct_im / 2.0;
  }

  ct_re = u_ct_re_tmp * ct->f264.re - u_ct_im_tmp * ct->f264.im;
  ct_im = u_ct_re_tmp * ct->f264.im + u_ct_im_tmp * ct->f264.re;
  if (ct_im == 0.0) {
    t1701_re = ct_re / 2.0;
    t1701_im = 0.0;
  } else if (ct_re == 0.0) {
    t1701_re = 0.0;
    t1701_im = ct_im / 2.0;
  } else {
    t1701_re = ct_re / 2.0;
    t1701_im = ct_im / 2.0;
  }

  ct_re = v_ct_re_tmp * ct->f265.re - v_ct_im_tmp * ct->f265.im;
  ct_im = v_ct_re_tmp * ct->f265.im + v_ct_im_tmp * ct->f265.re;
  if (ct_im == 0.0) {
    t1702_re = ct_re / 2.0;
    t1702_im = 0.0;
  } else if (ct_re == 0.0) {
    t1702_re = 0.0;
    t1702_im = ct_im / 2.0;
  } else {
    t1702_re = ct_re / 2.0;
    t1702_im = ct_im / 2.0;
  }

  ct_re = v_ct_re_tmp * ct->f266.re - v_ct_im_tmp * ct->f266.im;
  ct_im = v_ct_re_tmp * ct->f266.im + v_ct_im_tmp * ct->f266.re;
  if (ct_im == 0.0) {
    t1703_re = ct_re / 2.0;
    t1703_im = 0.0;
  } else if (ct_re == 0.0) {
    t1703_re = 0.0;
    t1703_im = ct_im / 2.0;
  } else {
    t1703_re = ct_re / 2.0;
    t1703_im = ct_im / 2.0;
  }

  ct_re = w_ct_re_tmp * ct->f265.re - w_ct_im_tmp * ct->f265.im;
  ct_im = w_ct_re_tmp * ct->f265.im + w_ct_im_tmp * ct->f265.re;
  if (ct_im == 0.0) {
    t1704_re = ct_re / 2.0;
    t1704_im = 0.0;
  } else if (ct_re == 0.0) {
    t1704_re = 0.0;
    t1704_im = ct_im / 2.0;
  } else {
    t1704_re = ct_re / 2.0;
    t1704_im = ct_im / 2.0;
  }

  ct_re = v_ct_re_tmp * ct->f267.re - v_ct_im_tmp * ct->f267.im;
  ct_im = v_ct_re_tmp * ct->f267.im + v_ct_im_tmp * ct->f267.re;
  if (ct_im == 0.0) {
    t1705_re = ct_re / 2.0;
    t1705_im = 0.0;
  } else if (ct_re == 0.0) {
    t1705_re = 0.0;
    t1705_im = ct_im / 2.0;
  } else {
    t1705_re = ct_re / 2.0;
    t1705_im = ct_im / 2.0;
  }

  ct_re = w_ct_re_tmp * ct->f266.re - w_ct_im_tmp * ct->f266.im;
  ct_im = w_ct_re_tmp * ct->f266.im + w_ct_im_tmp * ct->f266.re;
  if (ct_im == 0.0) {
    t1706_re = ct_re / 2.0;
    t1706_im = 0.0;
  } else if (ct_re == 0.0) {
    t1706_re = 0.0;
    t1706_im = ct_im / 2.0;
  } else {
    t1706_re = ct_re / 2.0;
    t1706_im = ct_im / 2.0;
  }

  ct_re = w_ct_re_tmp * ct->f267.re - w_ct_im_tmp * ct->f267.im;
  ct_im = w_ct_re_tmp * ct->f267.im + w_ct_im_tmp * ct->f267.re;
  if (ct_im == 0.0) {
    t1707_re = ct_re / 2.0;
    t1707_im = 0.0;
  } else if (ct_re == 0.0) {
    t1707_re = 0.0;
    t1707_im = ct_im / 2.0;
  } else {
    t1707_re = ct_re / 2.0;
    t1707_im = ct_im / 2.0;
  }

  ct_re = x_ct_re_tmp * ct->f268.re - x_ct_im_tmp * ct->f268.im;
  ct_im = x_ct_re_tmp * ct->f268.im + x_ct_im_tmp * ct->f268.re;
  if (ct_im == 0.0) {
    t1708_re = ct_re / 2.0;
    t1708_im = 0.0;
  } else if (ct_re == 0.0) {
    t1708_re = 0.0;
    t1708_im = ct_im / 2.0;
  } else {
    t1708_re = ct_re / 2.0;
    t1708_im = ct_im / 2.0;
  }

  ct_re = x_ct_re_tmp * ct->f269.re - x_ct_im_tmp * ct->f269.im;
  ct_im = x_ct_re_tmp * ct->f269.im + x_ct_im_tmp * ct->f269.re;
  if (ct_im == 0.0) {
    t1709_re = ct_re / 2.0;
    t1709_im = 0.0;
  } else if (ct_re == 0.0) {
    t1709_re = 0.0;
    t1709_im = ct_im / 2.0;
  } else {
    t1709_re = ct_re / 2.0;
    t1709_im = ct_im / 2.0;
  }

  ct_re = y_ct_re_tmp * ct->f265.re - y_ct_im_tmp * ct->f265.im;
  ct_im = y_ct_re_tmp * ct->f265.im + y_ct_im_tmp * ct->f265.re;
  if (ct_im == 0.0) {
    t1710_re = ct_re / 2.0;
    t1710_im = 0.0;
  } else if (ct_re == 0.0) {
    t1710_re = 0.0;
    t1710_im = ct_im / 2.0;
  } else {
    t1710_re = ct_re / 2.0;
    t1710_im = ct_im / 2.0;
  }

  ct_re = x_ct_re_tmp * ct->f270.re - x_ct_im_tmp * ct->f270.im;
  ct_im = x_ct_re_tmp * ct->f270.im + x_ct_im_tmp * ct->f270.re;
  if (ct_im == 0.0) {
    t1711_re = ct_re / 2.0;
    t1711_im = 0.0;
  } else if (ct_re == 0.0) {
    t1711_re = 0.0;
    t1711_im = ct_im / 2.0;
  } else {
    t1711_re = ct_re / 2.0;
    t1711_im = ct_im / 2.0;
  }

  ct_re = y_ct_re_tmp * ct->f266.re - y_ct_im_tmp * ct->f266.im;
  ct_im = y_ct_re_tmp * ct->f266.im + y_ct_im_tmp * ct->f266.re;
  if (ct_im == 0.0) {
    t1712_re = ct_re / 2.0;
    t1712_im = 0.0;
  } else if (ct_re == 0.0) {
    t1712_re = 0.0;
    t1712_im = ct_im / 2.0;
  } else {
    t1712_re = ct_re / 2.0;
    t1712_im = ct_im / 2.0;
  }

  ct_re = y_ct_re_tmp * ct->f267.re - y_ct_im_tmp * ct->f267.im;
  ct_im = y_ct_re_tmp * ct->f267.im + y_ct_im_tmp * ct->f267.re;
  if (ct_im == 0.0) {
    t1713_re = ct_re / 2.0;
    t1713_im = 0.0;
  } else if (ct_re == 0.0) {
    t1713_re = 0.0;
    t1713_im = ct_im / 2.0;
  } else {
    t1713_re = ct_re / 2.0;
    t1713_im = ct_im / 2.0;
  }

  ct_re = ab_ct_re_tmp * ct->f268.re - ab_ct_im_tmp * ct->f268.im;
  ct_im = ab_ct_re_tmp * ct->f268.im + ab_ct_im_tmp * ct->f268.re;
  if (ct_im == 0.0) {
    t1714_re = ct_re / 2.0;
    t1714_im = 0.0;
  } else if (ct_re == 0.0) {
    t1714_re = 0.0;
    t1714_im = ct_im / 2.0;
  } else {
    t1714_re = ct_re / 2.0;
    t1714_im = ct_im / 2.0;
  }

  ct_re = ab_ct_re_tmp * ct->f269.re - ab_ct_im_tmp * ct->f269.im;
  ct_im = ab_ct_re_tmp * ct->f269.im + ab_ct_im_tmp * ct->f269.re;
  if (ct_im == 0.0) {
    t1715_re = ct_re / 2.0;
    t1715_im = 0.0;
  } else if (ct_re == 0.0) {
    t1715_re = 0.0;
    t1715_im = ct_im / 2.0;
  } else {
    t1715_re = ct_re / 2.0;
    t1715_im = ct_im / 2.0;
  }

  ct_re = bb_ct_re_tmp * ct->f268.re - bb_ct_im_tmp * ct->f268.im;
  ct_im = bb_ct_re_tmp * ct->f268.im + bb_ct_im_tmp * ct->f268.re;
  if (ct_im == 0.0) {
    t1716_re = ct_re / 2.0;
    t1716_im = 0.0;
  } else if (ct_re == 0.0) {
    t1716_re = 0.0;
    t1716_im = ct_im / 2.0;
  } else {
    t1716_re = ct_re / 2.0;
    t1716_im = ct_im / 2.0;
  }

  ct_re = ab_ct_re_tmp * ct->f270.re - ab_ct_im_tmp * ct->f270.im;
  ct_im = ab_ct_re_tmp * ct->f270.im + ab_ct_im_tmp * ct->f270.re;
  if (ct_im == 0.0) {
    t1717_re = ct_re / 2.0;
    t1717_im = 0.0;
  } else if (ct_re == 0.0) {
    t1717_re = 0.0;
    t1717_im = ct_im / 2.0;
  } else {
    t1717_re = ct_re / 2.0;
    t1717_im = ct_im / 2.0;
  }

  ct_re = bb_ct_re_tmp * ct->f269.re - bb_ct_im_tmp * ct->f269.im;
  ct_im = bb_ct_re_tmp * ct->f269.im + bb_ct_im_tmp * ct->f269.re;
  if (ct_im == 0.0) {
    t1718_re = ct_re / 2.0;
    t1718_im = 0.0;
  } else if (ct_re == 0.0) {
    t1718_re = 0.0;
    t1718_im = ct_im / 2.0;
  } else {
    t1718_re = ct_re / 2.0;
    t1718_im = ct_im / 2.0;
  }

  ct_re = bb_ct_re_tmp * ct->f270.re - bb_ct_im_tmp * ct->f270.im;
  ct_im = bb_ct_re_tmp * ct->f270.im + bb_ct_im_tmp * ct->f270.re;
  if (ct_im == 0.0) {
    t1719_re = ct_re / 2.0;
    t1719_im = 0.0;
  } else if (ct_re == 0.0) {
    t1719_re = 0.0;
    t1719_im = ct_im / 2.0;
  } else {
    t1719_re = ct_re / 2.0;
    t1719_im = ct_im / 2.0;
  }

  ct_re = cb_ct_re_tmp * ct->f271.re - cb_ct_im_tmp * ct->f271.im;
  ct_im = cb_ct_re_tmp * ct->f271.im + cb_ct_im_tmp * ct->f271.re;
  if (ct_im == 0.0) {
    t1720_re = ct_re / 2.0;
    t1720_im = 0.0;
  } else if (ct_re == 0.0) {
    t1720_re = 0.0;
    t1720_im = ct_im / 2.0;
  } else {
    t1720_re = ct_re / 2.0;
    t1720_im = ct_im / 2.0;
  }

  ct_re = cb_ct_re_tmp * ct->f272.re - cb_ct_im_tmp * ct->f272.im;
  ct_im = cb_ct_re_tmp * ct->f272.im + cb_ct_im_tmp * ct->f272.re;
  if (ct_im == 0.0) {
    t1721_re = ct_re / 2.0;
    t1721_im = 0.0;
  } else if (ct_re == 0.0) {
    t1721_re = 0.0;
    t1721_im = ct_im / 2.0;
  } else {
    t1721_re = ct_re / 2.0;
    t1721_im = ct_im / 2.0;
  }

  ct_re = db_ct_re_tmp * ct->f268.re - db_ct_im_tmp * ct->f268.im;
  ct_im = db_ct_re_tmp * ct->f268.im + db_ct_im_tmp * ct->f268.re;
  if (ct_im == 0.0) {
    t1722_re = ct_re / 2.0;
    t1722_im = 0.0;
  } else if (ct_re == 0.0) {
    t1722_re = 0.0;
    t1722_im = ct_im / 2.0;
  } else {
    t1722_re = ct_re / 2.0;
    t1722_im = ct_im / 2.0;
  }

  ct_re = cb_ct_re_tmp * ct->f273.re - cb_ct_im_tmp * ct->f273.im;
  ct_im = cb_ct_re_tmp * ct->f273.im + cb_ct_im_tmp * ct->f273.re;
  if (ct_im == 0.0) {
    t1723_re = ct_re / 2.0;
    t1723_im = 0.0;
  } else if (ct_re == 0.0) {
    t1723_re = 0.0;
    t1723_im = ct_im / 2.0;
  } else {
    t1723_re = ct_re / 2.0;
    t1723_im = ct_im / 2.0;
  }

  ct_re = db_ct_re_tmp * ct->f269.re - db_ct_im_tmp * ct->f269.im;
  ct_im = db_ct_re_tmp * ct->f269.im + db_ct_im_tmp * ct->f269.re;
  if (ct_im == 0.0) {
    t1724_re = ct_re / 2.0;
    t1724_im = 0.0;
  } else if (ct_re == 0.0) {
    t1724_re = 0.0;
    t1724_im = ct_im / 2.0;
  } else {
    t1724_re = ct_re / 2.0;
    t1724_im = ct_im / 2.0;
  }

  ct_re = db_ct_re_tmp * ct->f270.re - db_ct_im_tmp * ct->f270.im;
  ct_im = db_ct_re_tmp * ct->f270.im + db_ct_im_tmp * ct->f270.re;
  if (ct_im == 0.0) {
    t1725_re = ct_re / 2.0;
    t1725_im = 0.0;
  } else if (ct_re == 0.0) {
    t1725_re = 0.0;
    t1725_im = ct_im / 2.0;
  } else {
    t1725_re = ct_re / 2.0;
    t1725_im = ct_im / 2.0;
  }

  ct_re = eb_ct_re_tmp * ct->f271.re - eb_ct_im_tmp * ct->f271.im;
  ct_im = eb_ct_re_tmp * ct->f271.im + eb_ct_im_tmp * ct->f271.re;
  if (ct_im == 0.0) {
    t1726_re = ct_re / 2.0;
    t1726_im = 0.0;
  } else if (ct_re == 0.0) {
    t1726_re = 0.0;
    t1726_im = ct_im / 2.0;
  } else {
    t1726_re = ct_re / 2.0;
    t1726_im = ct_im / 2.0;
  }

  ct_re = eb_ct_re_tmp * ct->f272.re - eb_ct_im_tmp * ct->f272.im;
  ct_im = eb_ct_re_tmp * ct->f272.im + eb_ct_im_tmp * ct->f272.re;
  if (ct_im == 0.0) {
    t1727_re = ct_re / 2.0;
    t1727_im = 0.0;
  } else if (ct_re == 0.0) {
    t1727_re = 0.0;
    t1727_im = ct_im / 2.0;
  } else {
    t1727_re = ct_re / 2.0;
    t1727_im = ct_im / 2.0;
  }

  ct_re = fb_ct_re_tmp * ct->f271.re - fb_ct_im_tmp * ct->f271.im;
  ct_im = fb_ct_re_tmp * ct->f271.im + fb_ct_im_tmp * ct->f271.re;
  if (ct_im == 0.0) {
    t1728_re = ct_re / 2.0;
    t1728_im = 0.0;
  } else if (ct_re == 0.0) {
    t1728_re = 0.0;
    t1728_im = ct_im / 2.0;
  } else {
    t1728_re = ct_re / 2.0;
    t1728_im = ct_im / 2.0;
  }

  ct_re = eb_ct_re_tmp * ct->f273.re - eb_ct_im_tmp * ct->f273.im;
  ct_im = eb_ct_re_tmp * ct->f273.im + eb_ct_im_tmp * ct->f273.re;
  if (ct_im == 0.0) {
    t1729_re = ct_re / 2.0;
    t1729_im = 0.0;
  } else if (ct_re == 0.0) {
    t1729_re = 0.0;
    t1729_im = ct_im / 2.0;
  } else {
    t1729_re = ct_re / 2.0;
    t1729_im = ct_im / 2.0;
  }

  ct_re = fb_ct_re_tmp * ct->f272.re - fb_ct_im_tmp * ct->f272.im;
  ct_im = fb_ct_re_tmp * ct->f272.im + fb_ct_im_tmp * ct->f272.re;
  if (ct_im == 0.0) {
    t1730_re = ct_re / 2.0;
    t1730_im = 0.0;
  } else if (ct_re == 0.0) {
    t1730_re = 0.0;
    t1730_im = ct_im / 2.0;
  } else {
    t1730_re = ct_re / 2.0;
    t1730_im = ct_im / 2.0;
  }

  ct_re = fb_ct_re_tmp * ct->f273.re - fb_ct_im_tmp * ct->f273.im;
  ct_im = fb_ct_re_tmp * ct->f273.im + fb_ct_im_tmp * ct->f273.re;
  if (ct_im == 0.0) {
    t1731_re = ct_re / 2.0;
    t1731_im = 0.0;
  } else if (ct_re == 0.0) {
    t1731_re = 0.0;
    t1731_im = ct_im / 2.0;
  } else {
    t1731_re = ct_re / 2.0;
    t1731_im = ct_im / 2.0;
  }

  ct_re = gb_ct_re_tmp * ct->f271.re - gb_ct_im_tmp * ct->f271.im;
  ct_im = gb_ct_re_tmp * ct->f271.im + gb_ct_im_tmp * ct->f271.re;
  if (ct_im == 0.0) {
    t1732_re = ct_re / 2.0;
    t1732_im = 0.0;
  } else if (ct_re == 0.0) {
    t1732_re = 0.0;
    t1732_im = ct_im / 2.0;
  } else {
    t1732_re = ct_re / 2.0;
    t1732_im = ct_im / 2.0;
  }

  ct_re = gb_ct_re_tmp * ct->f272.re - gb_ct_im_tmp * ct->f272.im;
  ct_im = gb_ct_re_tmp * ct->f272.im + gb_ct_im_tmp * ct->f272.re;
  if (ct_im == 0.0) {
    t1733_re = ct_re / 2.0;
    t1733_im = 0.0;
  } else if (ct_re == 0.0) {
    t1733_re = 0.0;
    t1733_im = ct_im / 2.0;
  } else {
    t1733_re = ct_re / 2.0;
    t1733_im = ct_im / 2.0;
  }

  ct_re = gb_ct_re_tmp * ct->f273.re - gb_ct_im_tmp * ct->f273.im;
  ct_im = gb_ct_re_tmp * ct->f273.im + gb_ct_im_tmp * ct->f273.re;
  if (ct_im == 0.0) {
    t1734_re = ct_re / 2.0;
    t1734_im = 0.0;
  } else if (ct_re == 0.0) {
    t1734_re = 0.0;
    t1734_im = ct_im / 2.0;
  } else {
    t1734_re = ct_re / 2.0;
    t1734_im = ct_im / 2.0;
  }

  ct_re = hb_ct_re_tmp * ct->f274.re - hb_ct_im_tmp * ct->f274.im;
  ct_im = hb_ct_re_tmp * ct->f274.im + hb_ct_im_tmp * ct->f274.re;
  if (ct_im == 0.0) {
    t1735_re = ct_re / 2.0;
    t1735_im = 0.0;
  } else if (ct_re == 0.0) {
    t1735_re = 0.0;
    t1735_im = ct_im / 2.0;
  } else {
    t1735_re = ct_re / 2.0;
    t1735_im = ct_im / 2.0;
  }

  ct_re = hb_ct_re_tmp * ct->f275.re - hb_ct_im_tmp * ct->f275.im;
  ct_im = hb_ct_re_tmp * ct->f275.im + hb_ct_im_tmp * ct->f275.re;
  if (ct_im == 0.0) {
    t1736_re = ct_re / 2.0;
    t1736_im = 0.0;
  } else if (ct_re == 0.0) {
    t1736_re = 0.0;
    t1736_im = ct_im / 2.0;
  } else {
    t1736_re = ct_re / 2.0;
    t1736_im = ct_im / 2.0;
  }

  ct_re = hb_ct_re_tmp * ct->f276.re - hb_ct_im_tmp * ct->f276.im;
  ct_im = hb_ct_re_tmp * ct->f276.im + hb_ct_im_tmp * ct->f276.re;
  if (ct_im == 0.0) {
    t1737_re = ct_re / 2.0;
    t1737_im = 0.0;
  } else if (ct_re == 0.0) {
    t1737_re = 0.0;
    t1737_im = ct_im / 2.0;
  } else {
    t1737_re = ct_re / 2.0;
    t1737_im = ct_im / 2.0;
  }

  ct_re = ib_ct_re_tmp * ct->f274.re - ib_ct_im_tmp * ct->f274.im;
  ct_im = ib_ct_re_tmp * ct->f274.im + ib_ct_im_tmp * ct->f274.re;
  if (ct_im == 0.0) {
    t1738_re = ct_re / 2.0;
    t1738_im = 0.0;
  } else if (ct_re == 0.0) {
    t1738_re = 0.0;
    t1738_im = ct_im / 2.0;
  } else {
    t1738_re = ct_re / 2.0;
    t1738_im = ct_im / 2.0;
  }

  ct_re = ib_ct_re_tmp * ct->f275.re - ib_ct_im_tmp * ct->f275.im;
  ct_im = ib_ct_re_tmp * ct->f275.im + ib_ct_im_tmp * ct->f275.re;
  if (ct_im == 0.0) {
    t1739_re = ct_re / 2.0;
    t1739_im = 0.0;
  } else if (ct_re == 0.0) {
    t1739_re = 0.0;
    t1739_im = ct_im / 2.0;
  } else {
    t1739_re = ct_re / 2.0;
    t1739_im = ct_im / 2.0;
  }

  ct_re = jb_ct_re_tmp * ct->f274.re - jb_ct_im_tmp * ct->f274.im;
  ct_im = jb_ct_re_tmp * ct->f274.im + jb_ct_im_tmp * ct->f274.re;
  if (ct_im == 0.0) {
    t1740_re = ct_re / 2.0;
    t1740_im = 0.0;
  } else if (ct_re == 0.0) {
    t1740_re = 0.0;
    t1740_im = ct_im / 2.0;
  } else {
    t1740_re = ct_re / 2.0;
    t1740_im = ct_im / 2.0;
  }

  ct_re = ib_ct_re_tmp * ct->f276.re - ib_ct_im_tmp * ct->f276.im;
  ct_im = ib_ct_re_tmp * ct->f276.im + ib_ct_im_tmp * ct->f276.re;
  if (ct_im == 0.0) {
    t1741_re = ct_re / 2.0;
    t1741_im = 0.0;
  } else if (ct_re == 0.0) {
    t1741_re = 0.0;
    t1741_im = ct_im / 2.0;
  } else {
    t1741_re = ct_re / 2.0;
    t1741_im = ct_im / 2.0;
  }

  ct_re = jb_ct_re_tmp * ct->f275.re - jb_ct_im_tmp * ct->f275.im;
  ct_im = jb_ct_re_tmp * ct->f275.im + jb_ct_im_tmp * ct->f275.re;
  if (ct_im == 0.0) {
    t1742_re = ct_re / 2.0;
    t1742_im = 0.0;
  } else if (ct_re == 0.0) {
    t1742_re = 0.0;
    t1742_im = ct_im / 2.0;
  } else {
    t1742_re = ct_re / 2.0;
    t1742_im = ct_im / 2.0;
  }

  ct_re = jb_ct_re_tmp * ct->f276.re - jb_ct_im_tmp * ct->f276.im;
  ct_im = jb_ct_re_tmp * ct->f276.im + jb_ct_im_tmp * ct->f276.re;
  if (ct_im == 0.0) {
    t1743_re = ct_re / 2.0;
    t1743_im = 0.0;
  } else if (ct_re == 0.0) {
    t1743_re = 0.0;
    t1743_im = ct_im / 2.0;
  } else {
    t1743_re = ct_re / 2.0;
    t1743_im = ct_im / 2.0;
  }

  ct_re = kb_ct_re_tmp * ct->f274.re - kb_ct_im_tmp * ct->f274.im;
  ct_im = kb_ct_re_tmp * ct->f274.im + kb_ct_im_tmp * ct->f274.re;
  if (ct_im == 0.0) {
    t1744_re = ct_re / 2.0;
    t1744_im = 0.0;
  } else if (ct_re == 0.0) {
    t1744_re = 0.0;
    t1744_im = ct_im / 2.0;
  } else {
    t1744_re = ct_re / 2.0;
    t1744_im = ct_im / 2.0;
  }

  ct_re = kb_ct_re_tmp * ct->f275.re - kb_ct_im_tmp * ct->f275.im;
  ct_im = kb_ct_re_tmp * ct->f275.im + kb_ct_im_tmp * ct->f275.re;
  if (ct_im == 0.0) {
    t1745_re = ct_re / 2.0;
    t1745_im = 0.0;
  } else if (ct_re == 0.0) {
    t1745_re = 0.0;
    t1745_im = ct_im / 2.0;
  } else {
    t1745_re = ct_re / 2.0;
    t1745_im = ct_im / 2.0;
  }

  ct_re = kb_ct_re_tmp * ct->f276.re - kb_ct_im_tmp * ct->f276.im;
  ct_im = kb_ct_re_tmp * ct->f276.im + kb_ct_im_tmp * ct->f276.re;
  if (ct_im == 0.0) {
    t1746_re = ct_re / 2.0;
    t1746_im = 0.0;
  } else if (ct_re == 0.0) {
    t1746_re = 0.0;
    t1746_im = ct_im / 2.0;
  } else {
    t1746_re = ct_re / 2.0;
    t1746_im = ct_im / 2.0;
  }

  memset(&c_gradient[0], 0, 9U * sizeof(creal_T));
  c_gradient[9].re = (ct->f125 - ct->f407.re) - ct->f410.re;
  c_gradient[9].im = (0.0 - ct->f407.im) - ct->f410.im;
  c_gradient[10].re = (ct->f129 - ct->f413.re) - ct->f416.re;
  c_gradient[10].im = (0.0 - ct->f413.im) - ct->f416.im;
  c_gradient[11].re = (ct->f133 - ct->f419.re) - ct->f422.re;
  c_gradient[11].im = (0.0 - ct->f419.im) - ct->f422.im;
  c_gradient[12].re = (ct->f141 - ct->f425.re) - ct->f430.re;
  c_gradient[12].im = (0.0 - ct->f425.im) - ct->f430.im;
  c_gradient[13].re = (ct->f149 - ct->f433.re) - ct->f436.re;
  c_gradient[13].im = (0.0 - ct->f433.im) - ct->f436.im;
  c_gradient[14].re = (ct->f157 - ct->f439.re) - ct->f443.re;
  c_gradient[14].im = (0.0 - ct->f439.im) - ct->f443.im;
  c_gradient[15].re = (ct->f165 - ct->f446.re) - ct->f449.re;
  c_gradient[15].im = (0.0 - ct->f446.im) - ct->f449.im;
  c_gradient[16].re = (ct->f173 - ct->f453.re) - ct->f456.re;
  c_gradient[16].im = (0.0 - ct->f453.im) - ct->f456.im;
  c_gradient[17].re = (ct->f189 - ct->f459.re) - ct->f463.re;
  c_gradient[17].im = (0.0 - ct->f459.im) - ct->f463.im;
  memset(&c_gradient[18], 0, 21U * sizeof(creal_T));
  c_gradient[39].re = (-ct->f125 + ct->f407.re) + ct->f410.re;
  c_gradient[39].im = ct->f407.im + ct->f410.im;
  c_gradient[40].re = (-ct->f129 + ct->f413.re) + ct->f416.re;
  c_gradient[40].im = ct->f413.im + ct->f416.im;
  c_gradient[41].re = (-ct->f133 + ct->f419.re) + ct->f422.re;
  c_gradient[41].im = ct->f419.im + ct->f422.im;
  c_gradient[42].re = (-ct->f141 + ct->f425.re) + ct->f430.re;
  c_gradient[42].im = ct->f425.im + ct->f430.im;
  c_gradient[43].re = (-ct->f149 + ct->f433.re) + ct->f436.re;
  c_gradient[43].im = ct->f433.im + ct->f436.im;
  c_gradient[44].re = (-ct->f157 + ct->f439.re) + ct->f443.re;
  c_gradient[44].im = ct->f439.im + ct->f443.im;
  c_gradient[45].re = (-ct->f165 + ct->f446.re) + ct->f449.re;
  c_gradient[45].im = ct->f446.im + ct->f449.im;
  c_gradient[46].re = (-ct->f173 + ct->f453.re) + ct->f456.re;
  c_gradient[46].im = ct->f453.im + ct->f456.im;
  c_gradient[47].re = (-ct->f189 + ct->f459.re) + ct->f463.re;
  c_gradient[47].im = ct->f459.im + ct->f463.im;
  memset(&c_gradient[48], 0, 21U * sizeof(creal_T));
  c_gradient[69].re = ((ct->f905 + ct->f126) - t1641_re) - t1653_re;
  c_gradient[69].im = (0.0 - t1641_im) - t1653_im;
  c_gradient[70].re = ((ct->f908 + ct->f130) - t1651_re) - t1665_re;
  c_gradient[70].im = (0.0 - t1651_im) - t1665_im;
  c_gradient[71].re = ((ct->f914 + ct->f137) - t1663_re) - t1677_re;
  c_gradient[71].im = (0.0 - t1663_im) - t1677_im;
  c_gradient[72].re = ((ct->f920 + ct->f145) - t1675_re) - t1689_re;
  c_gradient[72].im = (0.0 - t1675_im) - t1689_im;
  c_gradient[73].re = ((ct->f926 + ct->f153) - t1687_re) - t1701_re;
  c_gradient[73].im = (0.0 - t1687_im) - t1701_im;
  c_gradient[74].re = ((ct->f932 + ct->f161) - t1699_re) - t1713_re;
  c_gradient[74].im = (0.0 - t1699_im) - t1713_im;
  c_gradient[75].re = ((ct->f938 + ct->f169) - t1711_re) - t1725_re;
  c_gradient[75].im = (0.0 - t1711_im) - t1725_im;
  c_gradient[76].re = ((ct->f944 + ct->f177) - t1723_re) - t1734_re;
  c_gradient[76].im = (0.0 - t1723_im) - t1734_im;
  c_gradient[77].re = ((ct->f10 + ct->f190) - t1737_re) - t1746_re;
  c_gradient[77].im = (0.0 - t1737_im) - t1746_im;
  c_gradient[78].re = 0.0;
  c_gradient[78].im = 0.0;
  c_gradient[79].re = ((ct->f913 + ct->f202) - t1645_re) + t1647_re;
  c_gradient[79].im = (0.0 - t1645_im) + t1647_im;
  c_gradient[80].re = ((ct->f919 + ct->f206) - t1657_re) + t1659_re;
  c_gradient[80].im = (0.0 - t1657_im) + t1659_im;
  c_gradient[81].re = ((ct->f925 + ct->f210) - t1669_re) + t1671_re;
  c_gradient[81].im = (0.0 - t1669_im) + t1671_im;
  c_gradient[82].re = ((ct->f931 + ct->f214) - t1681_re) + t1683_re;
  c_gradient[82].im = (0.0 - t1681_im) + t1683_im;
  c_gradient[83].re = ((ct->f937 + ct->f218) - t1693_re) + t1695_re;
  c_gradient[83].im = (0.0 - t1693_im) + t1695_im;
  c_gradient[84].re = ((ct->f943 + ct->f222) - t1705_re) + t1707_re;
  c_gradient[84].im = (0.0 - t1705_im) + t1707_im;
  c_gradient[85].re = ((ct->f947 + ct->f224) - t1717_re) + t1719_re;
  c_gradient[85].im = (0.0 - t1717_im) + t1719_im;
  c_gradient[86].re = ((ct->f7 + ct->f226) - t1729_re) + t1731_re;
  c_gradient[86].im = (0.0 - t1729_im) + t1731_im;
  c_gradient[87].re = ((ct->f13 + ct->f230) - t1741_re) + t1743_re;
  c_gradient[87].im = (0.0 - t1741_im) + t1743_im;
  memset(&c_gradient[88], 0, 11U * sizeof(creal_T));
  c_gradient[99].re = ((ct->f34 + ct->f196) + t1641_re) + t1653_re;
  c_gradient[99].im = t1641_im + t1653_im;
  c_gradient[100].re = ((ct->f37 + ct->f198) + t1651_re) + t1665_re;
  c_gradient[100].im = t1651_im + t1665_im;
  c_gradient[101].re = ((ct->f43 + ct->f201) + t1663_re) + t1677_re;
  c_gradient[101].im = t1663_im + t1677_im;
  c_gradient[102].re = ((ct->f49 + ct->f205) + t1675_re) + t1689_re;
  c_gradient[102].im = t1675_im + t1689_im;
  c_gradient[103].re = ((ct->f55 + ct->f209) + t1687_re) + t1701_re;
  c_gradient[103].im = t1687_im + t1701_im;
  c_gradient[104].re = ((ct->f61 + ct->f213) + t1699_re) + t1713_re;
  c_gradient[104].im = t1699_im + t1713_im;
  c_gradient[105].re = ((ct->f67 + ct->f217) + t1711_re) + t1725_re;
  c_gradient[105].im = t1711_im + t1725_im;
  c_gradient[106].re = ((ct->f73 + ct->f221) + t1723_re) + t1734_re;
  c_gradient[106].im = t1723_im + t1734_im;
  c_gradient[107].re = ((ct->f82 + ct->f228) + t1737_re) + t1746_re;
  c_gradient[107].im = t1737_im + t1746_im;
  c_gradient[108].re = 0.0;
  c_gradient[108].im = 0.0;
  c_gradient[109].re = ((ct->f42 + ct->f139) + t1645_re) - t1647_re;
  c_gradient[109].im = t1645_im - t1647_im;
  c_gradient[110].re = ((ct->f48 + ct->f147) + t1657_re) - t1659_re;
  c_gradient[110].im = t1657_im - t1659_im;
  c_gradient[111].re = ((ct->f54 + ct->f155) + t1669_re) - t1671_re;
  c_gradient[111].im = t1669_im - t1671_im;
  c_gradient[112].re = ((ct->f60 + ct->f163) + t1681_re) - t1683_re;
  c_gradient[112].im = t1681_im - t1683_im;
  c_gradient[113].re = ((ct->f66 + ct->f171) + t1693_re) - t1695_re;
  c_gradient[113].im = t1693_im - t1695_im;
  c_gradient[114].re = ((ct->f72 + ct->f178) + t1705_re) - t1707_re;
  c_gradient[114].im = t1705_im - t1707_im;
  c_gradient[115].re = ((ct->f76 + ct->f182) + t1717_re) - t1719_re;
  c_gradient[115].im = t1717_im - t1719_im;
  c_gradient[116].re = ((ct->f79 + ct->f186) + t1729_re) - t1731_re;
  c_gradient[116].im = t1729_im - t1731_im;
  c_gradient[117].re = ((ct->f85 + ct->f194) + t1741_re) - t1743_re;
  c_gradient[117].im = t1741_im - t1743_im;
  memset(&c_gradient[118], 0, 19U * sizeof(creal_T));
  c_gradient[137].re = (ct->f123 - ct->f406.re) - ct->f409.re;
  c_gradient[137].im = (0.0 - ct->f406.im) - ct->f409.im;
  c_gradient[138].re = (ct->f127 - ct->f412.re) - ct->f415.re;
  c_gradient[138].im = (0.0 - ct->f412.im) - ct->f415.im;
  c_gradient[139].re = (ct->f131 - ct->f418.re) - ct->f421.re;
  c_gradient[139].im = (0.0 - ct->f418.im) - ct->f421.im;
  c_gradient[140].re = (ct->f138 - ct->f424.re) - ct->f427.re;
  c_gradient[140].im = (0.0 - ct->f424.im) - ct->f427.im;
  c_gradient[141].re = (ct->f146 - ct->f432.re) - ct->f435.re;
  c_gradient[141].im = (0.0 - ct->f432.im) - ct->f435.im;
  c_gradient[142].re = (ct->f154 - ct->f438.re) - ct->f442.re;
  c_gradient[142].im = (0.0 - ct->f438.im) - ct->f442.im;
  c_gradient[143].re = (ct->f162 - ct->f445.re) - ct->f448.re;
  c_gradient[143].im = (0.0 - ct->f445.im) - ct->f448.im;
  c_gradient[144].re = (ct->f170 - ct->f452.re) - ct->f455.re;
  c_gradient[144].im = (0.0 - ct->f452.im) - ct->f455.im;
  c_gradient[145].re = (ct->f187 - ct->f458.re) - ct->f461.re;
  c_gradient[145].im = (0.0 - ct->f458.im) - ct->f461.im;
  memset(&c_gradient[146], 0, 21U * sizeof(creal_T));
  c_gradient[167].re = (-ct->f123 + ct->f406.re) + ct->f409.re;
  c_gradient[167].im = ct->f406.im + ct->f409.im;
  c_gradient[168].re = (-ct->f127 + ct->f412.re) + ct->f415.re;
  c_gradient[168].im = ct->f412.im + ct->f415.im;
  c_gradient[169].re = (-ct->f131 + ct->f418.re) + ct->f421.re;
  c_gradient[169].im = ct->f418.im + ct->f421.im;
  c_gradient[170].re = (-ct->f138 + ct->f424.re) + ct->f427.re;
  c_gradient[170].im = ct->f424.im + ct->f427.im;
  c_gradient[171].re = (-ct->f146 + ct->f432.re) + ct->f435.re;
  c_gradient[171].im = ct->f432.im + ct->f435.im;
  c_gradient[172].re = (-ct->f154 + ct->f438.re) + ct->f442.re;
  c_gradient[172].im = ct->f438.im + ct->f442.im;
  c_gradient[173].re = (-ct->f162 + ct->f445.re) + ct->f448.re;
  c_gradient[173].im = ct->f445.im + ct->f448.im;
  c_gradient[174].re = (-ct->f170 + ct->f452.re) + ct->f455.re;
  c_gradient[174].im = ct->f452.im + ct->f455.im;
  c_gradient[175].re = (-ct->f187 + ct->f458.re) + ct->f461.re;
  c_gradient[175].im = ct->f458.im + ct->f461.im;
  memset(&c_gradient[176], 0, 21U * sizeof(creal_T));
  c_gradient[197].re = ((ct->f904 + ct->f124) - t1640_re) - t1652_re;
  c_gradient[197].im = (0.0 - t1640_im) - t1652_im;
  c_gradient[198].re = ((ct->f907 + ct->f128) - t1649_re) - t1664_re;
  c_gradient[198].im = (0.0 - t1649_im) - t1664_im;
  c_gradient[199].re = ((ct->f912 + ct->f132) - t1661_re) - t1676_re;
  c_gradient[199].im = (0.0 - t1661_im) - t1676_im;
  c_gradient[200].re = ((ct->f918 + ct->f140) - t1673_re) - t1688_re;
  c_gradient[200].im = (0.0 - t1673_im) - t1688_im;
  c_gradient[201].re = ((ct->f924 + ct->f148) - t1685_re) - t1700_re;
  c_gradient[201].im = (0.0 - t1685_im) - t1700_im;
  c_gradient[202].re = ((ct->f930 + ct->f156) - t1697_re) - t1712_re;
  c_gradient[202].im = (0.0 - t1697_im) - t1712_im;
  c_gradient[203].re = ((ct->f936 + ct->f164) - t1709_re) - t1724_re;
  c_gradient[203].im = (0.0 - t1709_im) - t1724_im;
  c_gradient[204].re = ((ct->f942 + ct->f172) - t1721_re) - t1733_re;
  c_gradient[204].im = (0.0 - t1721_im) - t1733_im;
  c_gradient[205].re = ((ct->f9 + ct->f188) - t1736_re) - t1745_re;
  c_gradient[205].im = (0.0 - t1736_im) - t1745_im;
  c_gradient[206].re = 0.0;
  c_gradient[206].im = 0.0;
  c_gradient[207].re = ((ct->f911 + ct->f200) - t1643_re) + t1646_re;
  c_gradient[207].im = (0.0 - t1643_im) + t1646_im;
  c_gradient[208].re = ((ct->f917 + ct->f204) - t1655_re) + t1658_re;
  c_gradient[208].im = (0.0 - t1655_im) + t1658_im;
  c_gradient[209].re = ((ct->f923 + ct->f208) - t1667_re) + t1670_re;
  c_gradient[209].im = (0.0 - t1667_im) + t1670_im;
  c_gradient[210].re = ((ct->f929 + ct->f212) - t1679_re) + t1682_re;
  c_gradient[210].im = (0.0 - t1679_im) + t1682_im;
  c_gradient[211].re = ((ct->f935 + ct->f216) - t1691_re) + t1694_re;
  c_gradient[211].im = (0.0 - t1691_im) + t1694_im;
  c_gradient[212].re = ((ct->f941 + ct->f220) - t1703_re) + t1706_re;
  c_gradient[212].im = (0.0 - t1703_im) + t1706_im;
  c_gradient[213].re = ((ct->f946 + ct->f223) - t1715_re) + t1718_re;
  c_gradient[213].im = (0.0 - t1715_im) + t1718_im;
  c_gradient[214].re = ((ct->f6 + ct->f225) - t1727_re) + t1730_re;
  c_gradient[214].im = (0.0 - t1727_im) + t1730_im;
  c_gradient[215].re = ((ct->f12 + ct->f229) - t1739_re) + t1742_re;
  c_gradient[215].im = (0.0 - t1739_im) + t1742_im;
  memset(&c_gradient[216], 0, 11U * sizeof(creal_T));
  c_gradient[227].re = ((ct->f33 + ct->f195) + t1640_re) + t1652_re;
  c_gradient[227].im = t1640_im + t1652_im;
  c_gradient[228].re = ((ct->f36 + ct->f197) + t1649_re) + t1664_re;
  c_gradient[228].im = t1649_im + t1664_im;
  c_gradient[229].re = ((ct->f41 + ct->f199) + t1661_re) + t1676_re;
  c_gradient[229].im = t1661_im + t1676_im;
  c_gradient[230].re = ((ct->f47 + ct->f203) + t1673_re) + t1688_re;
  c_gradient[230].im = t1673_im + t1688_im;
  c_gradient[231].re = ((ct->f53 + ct->f207) + t1685_re) + t1700_re;
  c_gradient[231].im = t1685_im + t1700_im;
  c_gradient[232].re = ((ct->f59 + ct->f211) + t1697_re) + t1712_re;
  c_gradient[232].im = t1697_im + t1712_im;
  c_gradient[233].re = ((ct->f65 + ct->f215) + t1709_re) + t1724_re;
  c_gradient[233].im = t1709_im + t1724_im;
  c_gradient[234].re = ((ct->f71 + ct->f219) + t1721_re) + t1733_re;
  c_gradient[234].im = t1721_im + t1733_im;
  c_gradient[235].re = ((ct->f81 + ct->f227) + t1736_re) + t1745_re;
  c_gradient[235].im = t1736_im + t1745_im;
  c_gradient[236].re = 0.0;
  c_gradient[236].im = 0.0;
  c_gradient[237].re = ((ct->f40 + ct->f135) + t1643_re) - t1646_re;
  c_gradient[237].im = t1643_im - t1646_im;
  c_gradient[238].re = ((ct->f46 + ct->f143) + t1655_re) - t1658_re;
  c_gradient[238].im = t1655_im - t1658_im;
  c_gradient[239].re = ((ct->f52 + ct->f151) + t1667_re) - t1670_re;
  c_gradient[239].im = t1667_im - t1670_im;
  c_gradient[240].re = ((ct->f58 + ct->f159) + t1679_re) - t1682_re;
  c_gradient[240].im = t1679_im - t1682_im;
  c_gradient[241].re = ((ct->f64 + ct->f167) + t1691_re) - t1694_re;
  c_gradient[241].im = t1691_im - t1694_im;
  c_gradient[242].re = ((ct->f70 + ct->f175) + t1703_re) - t1706_re;
  c_gradient[242].im = t1703_im - t1706_im;
  c_gradient[243].re = ((ct->f75 + ct->f180) + t1715_re) - t1718_re;
  c_gradient[243].im = t1715_im - t1718_im;
  c_gradient[244].re = ((ct->f78 + ct->f184) + t1727_re) - t1730_re;
  c_gradient[244].im = t1727_im - t1730_im;
  c_gradient[245].re = ((ct->f84 + ct->f192) + t1739_re) - t1742_re;
  c_gradient[245].im = t1739_im - t1742_im;
  memset(&c_gradient[246], 0, 19U * sizeof(creal_T));
  c_gradient[265].re = (ct->f86 - ct->f405.re) - ct->f408.re;
  c_gradient[265].im = (0.0 - ct->f405.im) - ct->f408.im;
  c_gradient[266].re = (ct->f87 - ct->f411.re) - ct->f414.re;
  c_gradient[266].im = (0.0 - ct->f411.im) - ct->f414.im;
  c_gradient[267].re = (ct->f88 - ct->f417.re) - ct->f420.re;
  c_gradient[267].im = (0.0 - ct->f417.im) - ct->f420.im;
  c_gradient[268].re = (ct->f90 - ct->f423.re) - ct->f426.re;
  c_gradient[268].im = (0.0 - ct->f423.im) - ct->f426.im;
  c_gradient[269].re = (ct->f92 - ct->f431.re) - ct->f434.re;
  c_gradient[269].im = (0.0 - ct->f431.im) - ct->f434.im;
  c_gradient[270].re = (ct->f94 - ct->f437.re) - ct->f441.re;
  c_gradient[270].im = (0.0 - ct->f437.im) - ct->f441.im;
  c_gradient[271].re = (ct->f96 - ct->f444.re) - ct->f447.re;
  c_gradient[271].im = (0.0 - ct->f444.im) - ct->f447.im;
  c_gradient[272].re = (ct->f98 - ct->f450.re) - ct->f454.re;
  c_gradient[272].im = (0.0 - ct->f450.im) - ct->f454.im;
  c_gradient[273].re = (ct->f102 - ct->f457.re) - ct->f460.re;
  c_gradient[273].im = (0.0 - ct->f457.im) - ct->f460.im;
  memset(&c_gradient[274], 0, 21U * sizeof(creal_T));
  c_gradient[295].re = (ct->f405.re + ct->f408.re) - ct->f556 * ct->f619 *
    ct->f663 * 6.0;
  c_gradient[295].im = ct->f405.im + ct->f408.im;
  c_gradient[296].re = (ct->f411.re + ct->f414.re) - ct->f561 * ct->f620 *
    ct->f665 * 6.0;
  c_gradient[296].im = ct->f411.im + ct->f414.im;
  c_gradient[297].re = (ct->f417.re + ct->f420.re) - ct->f564 * ct->f621 *
    ct->f667 * 6.0;
  c_gradient[297].im = ct->f417.im + ct->f420.im;
  c_gradient[298].re = (ct->f423.re + ct->f426.re) - ct->f568 * ct->f622 *
    ct->f670 * 6.0;
  c_gradient[298].im = ct->f423.im + ct->f426.im;
  c_gradient[299].re = (ct->f431.re + ct->f434.re) - ct->f572 * ct->f623 *
    ct->f672 * 6.0;
  c_gradient[299].im = ct->f431.im + ct->f434.im;
  c_gradient[300].re = (ct->f437.re + ct->f441.re) - ct->f586 * ct->f624 *
    ct->f674 * 6.0;
  c_gradient[300].im = ct->f437.im + ct->f441.im;
  c_gradient[301].re = (ct->f444.re + ct->f447.re) - ct->f625 * ct->f626 *
    ct->f676 * 6.0;
  c_gradient[301].im = ct->f444.im + ct->f447.im;
  c_gradient[302].re = (ct->f450.re + ct->f454.re) - ct->f659 * ct->f627 *
    ct->f678 * 6.0;
  c_gradient[302].im = ct->f450.im + ct->f454.im;
  c_gradient[303].re = (ct->f457.re + ct->f460.re) - ct->f693 * ct->f637 *
    ct->f681 * 6.0;
  c_gradient[303].im = ct->f457.im + ct->f460.im;
  memset(&c_gradient[304], 0, 21U * sizeof(creal_T));
  c_gradient[325].re = ((ct->f868 + ct->f903) - t1639_re) - t1650_re;
  c_gradient[325].im = (0.0 - t1639_im) - t1650_im;
  c_gradient[326].re = ((ct->f870 + ct->f906) - t1648_re) - t1662_re;
  c_gradient[326].im = (0.0 - t1648_im) - t1662_im;
  c_gradient[327].re = ((ct->f872 + ct->f910) - t1660_re) - t1674_re;
  c_gradient[327].im = (0.0 - t1660_im) - t1674_im;
  c_gradient[328].re = ((ct->f876 + ct->f916) - t1672_re) - t1686_re;
  c_gradient[328].im = (0.0 - t1672_im) - t1686_im;
  c_gradient[329].re = ((ct->f880 + ct->f922) - t1684_re) - t1698_re;
  c_gradient[329].im = (0.0 - t1684_im) - t1698_im;
  c_gradient[330].re = ((ct->f884 + ct->f928) - t1696_re) - t1710_re;
  c_gradient[330].im = (0.0 - t1696_im) - t1710_im;
  c_gradient[331].re = ((ct->f888 + ct->f934) - t1708_re) - t1722_re;
  c_gradient[331].im = (0.0 - t1708_im) - t1722_im;
  c_gradient[332].re = ((ct->f892 + ct->f940) - t1720_re) - t1732_re;
  c_gradient[332].im = (0.0 - t1720_im) - t1732_im;
  c_gradient[333].re = ((ct->f900 + ct->f8) - t1735_re) - t1744_re;
  c_gradient[333].im = (0.0 - t1735_im) - t1744_im;
  c_gradient[334].re = 0.0;
  c_gradient[334].im = 0.0;
  c_gradient[335].re = ((ct->f909 + ct->f17) - t1642_re) + t1644_re;
  c_gradient[335].im = (0.0 - t1642_im) + t1644_im;
  c_gradient[336].re = ((ct->f915 + ct->f19) - t1654_re) + t1656_re;
  c_gradient[336].im = (0.0 - t1654_im) + t1656_im;
  c_gradient[337].re = ((ct->f921 + ct->f21) - t1666_re) + t1668_re;
  c_gradient[337].im = (0.0 - t1666_im) + t1668_im;
  c_gradient[338].re = ((ct->f927 + ct->f23) - t1678_re) + t1680_re;
  c_gradient[338].im = (0.0 - t1678_im) + t1680_im;
  c_gradient[339].re = ((ct->f933 + ct->f25) - t1690_re) + t1692_re;
  c_gradient[339].im = (0.0 - t1690_im) + t1692_im;
  c_gradient[340].re = ((ct->f939 + ct->f27) - t1702_re) + t1704_re;
  c_gradient[340].im = (0.0 - t1702_im) + t1704_im;
  c_gradient[341].re = ((ct->f945 + ct->f28) - t1714_re) + t1716_re;
  c_gradient[341].im = (0.0 - t1714_im) + t1716_im;
  c_gradient[342].re = ((ct->f5 + ct->f29) - t1726_re) + t1728_re;
  c_gradient[342].im = (0.0 - t1726_im) + t1728_im;
  c_gradient[343].re = ((ct->f11 + ct->f31) - t1738_re) + t1740_re;
  c_gradient[343].im = (0.0 - t1738_im) + t1740_im;
  memset(&c_gradient[344], 0, 11U * sizeof(creal_T));
  c_gradient[355].re = ((ct->f14 + ct->f32) + t1639_re) + t1650_re;
  c_gradient[355].im = t1639_im + t1650_im;
  c_gradient[356].re = ((ct->f15 + ct->f35) + t1648_re) + t1662_re;
  c_gradient[356].im = t1648_im + t1662_im;
  c_gradient[357].re = ((ct->f16 + ct->f39) + t1660_re) + t1674_re;
  c_gradient[357].im = t1660_im + t1674_im;
  c_gradient[358].re = ((ct->f18 + ct->f45) + t1672_re) + t1686_re;
  c_gradient[358].im = t1672_im + t1686_im;
  c_gradient[359].re = ((ct->f20 + ct->f51) + t1684_re) + t1698_re;
  c_gradient[359].im = t1684_im + t1698_im;
  c_gradient[360].re = ((ct->f22 + ct->f57) + t1696_re) + t1710_re;
  c_gradient[360].im = t1696_im + t1710_im;
  c_gradient[361].re = ((ct->f24 + ct->f63) + t1708_re) + t1722_re;
  c_gradient[361].im = t1708_im + t1722_im;
  c_gradient[362].re = ((ct->f26 + ct->f69) + t1720_re) + t1732_re;
  c_gradient[362].im = t1720_im + t1732_im;
  c_gradient[363].re = ((ct->f30 + ct->f80) + t1735_re) + t1744_re;
  c_gradient[363].im = t1735_im + t1744_im;
  c_gradient[364].re = 0.0;
  c_gradient[364].im = 0.0;
  c_gradient[365].re = ((ct->f874 + ct->f38) + t1642_re) - t1644_re;
  c_gradient[365].im = t1642_im - t1644_im;
  c_gradient[366].re = ((ct->f878 + ct->f44) + t1654_re) - t1656_re;
  c_gradient[366].im = t1654_im - t1656_im;
  c_gradient[367].re = ((ct->f882 + ct->f50) + t1666_re) - t1668_re;
  c_gradient[367].im = t1666_im - t1668_im;
  c_gradient[368].re = ((ct->f886 + ct->f56) + t1678_re) - t1680_re;
  c_gradient[368].im = t1678_im - t1680_im;
  c_gradient[369].re = ((ct->f890 + ct->f62) + t1690_re) - t1692_re;
  c_gradient[369].im = t1690_im - t1692_im;
  c_gradient[370].re = ((ct->f894 + ct->f68) + t1702_re) - t1704_re;
  c_gradient[370].im = t1702_im - t1704_im;
  c_gradient[371].re = ((ct->f896 + ct->f74) + t1714_re) - t1716_re;
  c_gradient[371].im = t1714_im - t1716_im;
  c_gradient[372].re = ((ct->f898 + ct->f77) + t1726_re) - t1728_re;
  c_gradient[372].im = t1726_im - t1728_im;
  c_gradient[373].re = ((ct->f902 + ct->f83) + t1738_re) - t1740_re;
  c_gradient[373].im = t1738_im - t1740_im;
  memset(&c_gradient[374], 0, 19U * sizeof(creal_T));
  c_gradient[393].re = (ct->f867 - ct->f387.re) - ct->f388.re;
  c_gradient[393].im = (0.0 - ct->f387.im) - ct->f388.im;
  c_gradient[394].re = (ct->f869 - ct->f389.re) - ct->f390.re;
  c_gradient[394].im = (0.0 - ct->f389.im) - ct->f390.im;
  c_gradient[395].re = (ct->f871 - ct->f391.re) - ct->f392.re;
  c_gradient[395].im = (0.0 - ct->f391.im) - ct->f392.im;
  c_gradient[396].re = (ct->f875 - ct->f393.re) - ct->f394.re;
  c_gradient[396].im = (0.0 - ct->f393.im) - ct->f394.im;
  c_gradient[397].re = (ct->f879 - ct->f395.re) - ct->f396.re;
  c_gradient[397].im = (0.0 - ct->f395.im) - ct->f396.im;
  c_gradient[398].re = (ct->f883 - ct->f397.re) - ct->f398.re;
  c_gradient[398].im = (0.0 - ct->f397.im) - ct->f398.im;
  c_gradient[399].re = (ct->f887 - ct->f399.re) - ct->f400.re;
  c_gradient[399].im = (0.0 - ct->f399.im) - ct->f400.im;
  c_gradient[400].re = (ct->f891 - ct->f401.re) - ct->f402.re;
  c_gradient[400].im = (0.0 - ct->f401.im) - ct->f402.im;
  c_gradient[401].re = (ct->f899 - ct->f403.re) - ct->f404.re;
  c_gradient[401].im = (0.0 - ct->f403.im) - ct->f404.im;
  memset(&c_gradient[402], 0, 21U * sizeof(creal_T));
  c_gradient[423].re = (-ct->f867 + ct->f387.re) + ct->f388.re;
  c_gradient[423].im = ct->f387.im + ct->f388.im;
  c_gradient[424].re = (-ct->f869 + ct->f389.re) + ct->f390.re;
  c_gradient[424].im = ct->f389.im + ct->f390.im;
  c_gradient[425].re = (-ct->f871 + ct->f391.re) + ct->f392.re;
  c_gradient[425].im = ct->f391.im + ct->f392.im;
  c_gradient[426].re = (-ct->f875 + ct->f393.re) + ct->f394.re;
  c_gradient[426].im = ct->f393.im + ct->f394.im;
  c_gradient[427].re = (-ct->f879 + ct->f395.re) + ct->f396.re;
  c_gradient[427].im = ct->f395.im + ct->f396.im;
  c_gradient[428].re = (-ct->f883 + ct->f397.re) + ct->f398.re;
  c_gradient[428].im = ct->f397.im + ct->f398.im;
  c_gradient[429].re = (-ct->f887 + ct->f399.re) + ct->f400.re;
  c_gradient[429].im = ct->f399.im + ct->f400.im;
  c_gradient[430].re = (-ct->f891 + ct->f401.re) + ct->f402.re;
  c_gradient[430].im = ct->f401.im + ct->f402.im;
  c_gradient[431].re = (-ct->f899 + ct->f403.re) + ct->f404.re;
  c_gradient[431].im = ct->f403.im + ct->f404.im;
  memset(&c_gradient[432], 0, 20U * sizeof(creal_T));
  c_gradient[452].re = ct->f578;
  c_gradient[452].im = 0.0;
  c_gradient[453].re = ((ct->f794 + ct->f812) - t1603_re) - t1607_re;
  c_gradient[453].im = (0.0 - t1603_im) - t1607_im;
  c_gradient[454].re = ((ct->f795 + ct->f813) - t1606_re) - t1611_re;
  c_gradient[454].im = (0.0 - t1606_im) - t1611_im;
  c_gradient[455].re = ((ct->f797 + ct->f814) - t1610_re) - t1615_re;
  c_gradient[455].im = (0.0 - t1610_im) - t1615_im;
  c_gradient[456].re = ((ct->f799 + ct->f816) - t1614_re) - t1619_re;
  c_gradient[456].im = (0.0 - t1614_im) - t1619_im;
  c_gradient[457].re = ((ct->f801 + ct->f818) - t1618_re) - t1623_re;
  c_gradient[457].im = (0.0 - t1618_im) - t1623_im;
  c_gradient[458].re = ((ct->f803 + ct->f820) - t1622_re) - t1627_re;
  c_gradient[458].im = (0.0 - t1622_im) - t1627_im;
  c_gradient[459].re = ((ct->f805 + ct->f822) - t1626_re) - t1631_re;
  c_gradient[459].im = (0.0 - t1626_im) - t1631_im;
  c_gradient[460].re = ((ct->f807 + ct->f824) - t1630_re) - t1634_re;
  c_gradient[460].im = (0.0 - t1630_im) - t1634_im;
  c_gradient[461].re = ((ct->f810 + ct->f828) - t1635_re) - t1638_re;
  c_gradient[461].im = (0.0 - t1635_im) - t1638_im;
  c_gradient[462].re = ct->f584;
  c_gradient[462].im = 0.0;
  c_gradient[463].re = ((ct->f796 + ct->f851) - t1604_re) + t1605_re;
  c_gradient[463].im = (0.0 - t1604_im) + t1605_im;
  c_gradient[464].re = ((ct->f798 + ct->f853) - t1608_re) + t1609_re;
  c_gradient[464].im = (0.0 - t1608_im) + t1609_im;
  c_gradient[465].re = ((ct->f800 + ct->f855) - t1612_re) + t1613_re;
  c_gradient[465].im = (0.0 - t1612_im) + t1613_im;
  c_gradient[466].re = ((ct->f802 + ct->f857) - t1616_re) + t1617_re;
  c_gradient[466].im = (0.0 - t1616_im) + t1617_im;
  c_gradient[467].re = ((ct->f804 + ct->f859) - t1620_re) + t1621_re;
  c_gradient[467].im = (0.0 - t1620_im) + t1621_im;
  c_gradient[468].re = ((ct->f806 + ct->f861) - t1624_re) + t1625_re;
  c_gradient[468].im = (0.0 - t1624_im) + t1625_im;
  c_gradient[469].re = ((ct->f808 + ct->f862) - t1628_re) + t1629_re;
  c_gradient[469].im = (0.0 - t1628_im) + t1629_im;
  c_gradient[470].re = ((ct->f809 + ct->f863) - t1632_re) + t1633_re;
  c_gradient[470].im = (0.0 - t1632_im) + t1633_im;
  c_gradient[471].re = ((ct->f811 + ct->f866) - t1636_re) + t1637_re;
  c_gradient[471].im = (0.0 - t1636_im) + t1637_im;
  memset(&c_gradient[472], 0, 10U * sizeof(creal_T));
  c_gradient[482].re = ct->f582;
  c_gradient[482].im = 0.0;
  c_gradient[483].re = ((ct->f830 + ct->f848) + t1603_re) + t1607_re;
  c_gradient[483].im = t1603_im + t1607_im;
  c_gradient[484].re = ((ct->f831 + ct->f849) + t1606_re) + t1611_re;
  c_gradient[484].im = t1606_im + t1611_im;
  c_gradient[485].re = ((ct->f833 + ct->f850) + t1610_re) + t1615_re;
  c_gradient[485].im = t1610_im + t1615_im;
  c_gradient[486].re = ((ct->f835 + ct->f852) + t1614_re) + t1619_re;
  c_gradient[486].im = t1614_im + t1619_im;
  c_gradient[487].re = ((ct->f837 + ct->f854) + t1618_re) + t1623_re;
  c_gradient[487].im = t1618_im + t1623_im;
  c_gradient[488].re = ((ct->f839 + ct->f856) + t1622_re) + t1627_re;
  c_gradient[488].im = t1622_im + t1627_im;
  c_gradient[489].re = ((ct->f841 + ct->f858) + t1626_re) + t1631_re;
  c_gradient[489].im = t1626_im + t1631_im;
  c_gradient[490].re = ((ct->f843 + ct->f860) + t1630_re) + t1634_re;
  c_gradient[490].im = t1630_im + t1634_im;
  c_gradient[491].re = ((ct->f846 + ct->f864) + t1635_re) + t1638_re;
  c_gradient[491].im = t1635_im + t1638_im;
  c_gradient[492].re = ct->f580;
  c_gradient[492].im = 0.0;
  c_gradient[493].re = ((ct->f815 + ct->f832) + t1604_re) - t1605_re;
  c_gradient[493].im = t1604_im - t1605_im;
  c_gradient[494].re = ((ct->f817 + ct->f834) + t1608_re) - t1609_re;
  c_gradient[494].im = t1608_im - t1609_im;
  c_gradient[495].re = ((ct->f819 + ct->f836) + t1612_re) - t1613_re;
  c_gradient[495].im = t1612_im - t1613_im;
  c_gradient[496].re = ((ct->f821 + ct->f838) + t1616_re) - t1617_re;
  c_gradient[496].im = t1616_im - t1617_im;
  c_gradient[497].re = ((ct->f823 + ct->f840) + t1620_re) - t1621_re;
  c_gradient[497].im = t1620_im - t1621_im;
  c_gradient[498].re = ((ct->f825 + ct->f842) + t1624_re) - t1625_re;
  c_gradient[498].im = t1624_im - t1625_im;
  c_gradient[499].re = ((ct->f826 + ct->f844) + t1628_re) - t1629_re;
  c_gradient[499].im = t1628_im - t1629_im;
  c_gradient[500].re = ((ct->f827 + ct->f845) + t1632_re) - t1633_re;
  c_gradient[500].im = t1632_im - t1633_im;
  c_gradient[501].re = ((ct->f829 + ct->f847) + t1636_re) - t1637_re;
  c_gradient[501].im = t1636_im - t1637_im;
  memset(&c_gradient[502], 0, 18U * sizeof(creal_T));
  c_gradient[520].re = (ct->f578 - ct->f585) - ct->f588;
  c_gradient[520].im = 0.0;
  c_gradient[521].re = (ct->f104 - ct->f277.re) - ct->f278.re;
  c_gradient[521].im = (0.0 - ct->f277.im) - ct->f278.im;
  c_gradient[522].re = (ct->f106 - ct->f280.re) - ct->f282.re;
  c_gradient[522].im = (0.0 - ct->f280.im) - ct->f282.im;
  c_gradient[523].re = (ct->f107 - ct->f284.re) - ct->f286.re;
  c_gradient[523].im = (0.0 - ct->f284.im) - ct->f286.im;
  c_gradient[524].re = (ct->f109 - ct->f288.re) - ct->f290.re;
  c_gradient[524].im = (0.0 - ct->f288.im) - ct->f290.im;
  c_gradient[525].re = (ct->f111 - ct->f292.re) - ct->f294.re;
  c_gradient[525].im = (0.0 - ct->f292.im) - ct->f294.im;
  c_gradient[526].re = (ct->f113 - ct->f296.re) - ct->f298.re;
  c_gradient[526].im = (0.0 - ct->f296.im) - ct->f298.im;
  c_gradient[527].re = (ct->f115 - ct->f300.re) - ct->f302.re;
  c_gradient[527].im = (0.0 - ct->f300.im) - ct->f302.im;
  c_gradient[528].re = (ct->f117 - ct->f304.re) - ct->f306.re;
  c_gradient[528].im = (0.0 - ct->f304.im) - ct->f306.im;
  c_gradient[529].re = (ct->f121 - ct->f309.re) - ct->f310.re;
  c_gradient[529].im = (0.0 - ct->f309.im) - ct->f310.im;
  memset(&c_gradient[530], 0, 20U * sizeof(creal_T));
  c_gradient[550].re = (ct->f582 + ct->f585) + ct->f588;
  c_gradient[550].im = 0.0;
  c_gradient[551].re = (-ct->f104 + ct->f277.re) + ct->f278.re;
  c_gradient[551].im = ct->f277.im + ct->f278.im;
  c_gradient[552].re = (-ct->f106 + ct->f280.re) + ct->f282.re;
  c_gradient[552].im = ct->f280.im + ct->f282.im;
  c_gradient[553].re = (-ct->f107 + ct->f284.re) + ct->f286.re;
  c_gradient[553].im = ct->f284.im + ct->f286.im;
  c_gradient[554].re = (-ct->f109 + ct->f288.re) + ct->f290.re;
  c_gradient[554].im = ct->f288.im + ct->f290.im;
  c_gradient[555].re = (-ct->f111 + ct->f292.re) + ct->f294.re;
  c_gradient[555].im = ct->f292.im + ct->f294.im;
  c_gradient[556].re = (-ct->f113 + ct->f296.re) + ct->f298.re;
  c_gradient[556].im = ct->f296.im + ct->f298.im;
  c_gradient[557].re = (-ct->f115 + ct->f300.re) + ct->f302.re;
  c_gradient[557].im = ct->f300.im + ct->f302.im;
  c_gradient[558].re = (-ct->f117 + ct->f304.re) + ct->f306.re;
  c_gradient[558].im = ct->f304.im + ct->f306.im;
  c_gradient[559].re = (-ct->f121 + ct->f309.re) + ct->f310.re;
  c_gradient[559].im = ct->f309.im + ct->f310.im;
  memset(&c_gradient[560], 0, 20U * sizeof(creal_T));
  c_gradient[580].re = (ct->f577 + ct->f598) - ct->f2 * ct->f865 * ct->f575 *
    2.0;
  c_gradient[580].im = 0.0;
  c_gradient[581].re = (ct->f758 - ct->f313.re) - ct->f318.re;
  c_gradient[581].im = (0.0 - ct->f313.im) - ct->f318.im;
  c_gradient[582].re = (ct->f759 - ct->f317.re) - ct->f328.re;
  c_gradient[582].im = (0.0 - ct->f317.im) - ct->f328.im;
  c_gradient[583].re = (ct->f760 - ct->f324.re) - ct->f336.re;
  c_gradient[583].im = (0.0 - ct->f324.im) - ct->f336.im;
  c_gradient[584].re = (ct->f762 - ct->f334.re) - ct->f344.re;
  c_gradient[584].im = (0.0 - ct->f334.im) - ct->f344.im;
  c_gradient[585].re = (ct->f764 - ct->f342.re) - ct->f352.re;
  c_gradient[585].im = (0.0 - ct->f342.im) - ct->f352.im;
  c_gradient[586].re = (ct->f766 - ct->f350.re) - ct->f360.re;
  c_gradient[586].im = (0.0 - ct->f350.im) - ct->f360.im;
  c_gradient[587].re = (ct->f768 - ct->f358.re) - ct->f368.re;
  c_gradient[587].im = (0.0 - ct->f358.im) - ct->f368.im;
  c_gradient[588].re = (ct->f770 - ct->f366.re) - ct->f375.re;
  c_gradient[588].im = (0.0 - ct->f366.im) - ct->f375.im;
  c_gradient[589].re = (ct->f774 - ct->f379.re) - ct->f383.re;
  c_gradient[589].im = (0.0 - ct->f379.im) - ct->f383.im;
  c_gradient[590].re = (ct->f579 + ct->f590) - ct->f1 * ct->f865 * ct->f575 *
    2.0;
  c_gradient[590].im = 0.0;
  c_gradient[591].re = (ct->f761 - ct->f314.re) + ct->f315.re;
  c_gradient[591].im = (0.0 - ct->f314.im) + ct->f315.im;
  c_gradient[592].re = (ct->f763 - ct->f320.re) + ct->f322.re;
  c_gradient[592].im = (0.0 - ct->f320.im) + ct->f322.im;
  c_gradient[593].re = (ct->f765 - ct->f330.re) + ct->f332.re;
  c_gradient[593].im = (0.0 - ct->f330.im) + ct->f332.im;
  c_gradient[594].re = (ct->f767 - ct->f338.re) + ct->f340.re;
  c_gradient[594].im = (0.0 - ct->f338.im) + ct->f340.im;
  c_gradient[595].re = (ct->f769 - ct->f346.re) + ct->f348.re;
  c_gradient[595].im = (0.0 - ct->f346.im) + ct->f348.im;
  c_gradient[596].re = (ct->f771 - ct->f354.re) + ct->f356.re;
  c_gradient[596].im = (0.0 - ct->f354.im) + ct->f356.im;
  c_gradient[597].re = (ct->f772 - ct->f362.re) + ct->f364.re;
  c_gradient[597].im = (0.0 - ct->f362.im) + ct->f364.im;
  c_gradient[598].re = (ct->f773 - ct->f370.re) + ct->f372.re;
  c_gradient[598].im = (0.0 - ct->f370.im) + ct->f372.im;
  c_gradient[599].re = (ct->f775 - ct->f380.re) + ct->f381.re;
  c_gradient[599].im = (0.0 - ct->f380.im) + ct->f381.im;
  memset(&c_gradient[600], 0, 10U * sizeof(creal_T));
  c_gradient[610].re = (ct->f581 + ct->f591) + ct->f593;
  c_gradient[610].im = 0.0;
  c_gradient[611].re = (ct->f776 + ct->f313.re) + ct->f318.re;
  c_gradient[611].im = ct->f313.im + ct->f318.im;
  c_gradient[612].re = (ct->f777 + ct->f317.re) + ct->f328.re;
  c_gradient[612].im = ct->f317.im + ct->f328.im;
  c_gradient[613].re = (ct->f778 + ct->f324.re) + ct->f336.re;
  c_gradient[613].im = ct->f324.im + ct->f336.im;
  c_gradient[614].re = (ct->f780 + ct->f334.re) + ct->f344.re;
  c_gradient[614].im = ct->f334.im + ct->f344.im;
  c_gradient[615].re = (ct->f782 + ct->f342.re) + ct->f352.re;
  c_gradient[615].im = ct->f342.im + ct->f352.im;
  c_gradient[616].re = (ct->f784 + ct->f350.re) + ct->f360.re;
  c_gradient[616].im = ct->f350.im + ct->f360.im;
  c_gradient[617].re = (ct->f786 + ct->f358.re) + ct->f368.re;
  c_gradient[617].im = ct->f358.im + ct->f368.im;
  c_gradient[618].re = (ct->f788 + ct->f366.re) + ct->f375.re;
  c_gradient[618].im = ct->f366.im + ct->f375.im;
  c_gradient[619].re = (ct->f792 + ct->f379.re) + ct->f383.re;
  c_gradient[619].im = ct->f379.im + ct->f383.im;
  c_gradient[620].re = (ct->f583 + ct->f595) + ct->f597;
  c_gradient[620].im = 0.0;
  c_gradient[621].re = (ct->f779 + ct->f314.re) - ct->f315.re;
  c_gradient[621].im = ct->f314.im - ct->f315.im;
  c_gradient[622].re = (ct->f781 + ct->f320.re) - ct->f322.re;
  c_gradient[622].im = ct->f320.im - ct->f322.im;
  c_gradient[623].re = (ct->f783 + ct->f330.re) - ct->f332.re;
  c_gradient[623].im = ct->f330.im - ct->f332.im;
  c_gradient[624].re = (ct->f785 + ct->f338.re) - ct->f340.re;
  c_gradient[624].im = ct->f338.im - ct->f340.im;
  c_gradient[625].re = (ct->f787 + ct->f346.re) - ct->f348.re;
  c_gradient[625].im = ct->f346.im - ct->f348.im;
  c_gradient[626].re = (ct->f789 + ct->f354.re) - ct->f356.re;
  c_gradient[626].im = ct->f354.im - ct->f356.im;
  c_gradient[627].re = (ct->f790 + ct->f362.re) - ct->f364.re;
  c_gradient[627].im = ct->f362.im - ct->f364.im;
  c_gradient[628].re = (ct->f791 + ct->f370.re) - ct->f372.re;
  c_gradient[628].im = ct->f370.im - ct->f372.im;
  c_gradient[629].re = (ct->f793 + ct->f380.re) - ct->f381.re;
  c_gradient[629].im = ct->f380.im - ct->f381.im;
  memset(&c_gradient[630], 0, 147U * sizeof(creal_T));
  t1691_re = ct->f639 * ct->f664;
  t1646_im = t1691_re * ct->f231.re;
  t1647_re = t1691_re * ct->f231.im;
  t1691_re = ct->f696.re - ct->f733.re;
  t1647_im = ct->f696.im - ct->f733.im;
  ct_re = t1646_im * t1691_re - t1647_re * t1647_im;
  ct_im = t1646_im * t1647_im + t1647_re * t1691_re;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1648_re = ct->f648 * ct->f664;
  t1648_im = t1648_re * ct->f231.re;
  t1649_re = t1648_re * ct->f231.im;
  t1746_im = t1648_im * t1691_re - t1649_re * t1647_im;
  t1746_re = t1648_im * t1647_im + t1649_re * t1691_re;
  if (t1746_re == 0.0) {
    t1746_im /= 2.0;
    t1746_re = 0.0;
  } else if (t1746_im == 0.0) {
    t1746_im = 0.0;
    t1746_re /= 2.0;
  } else {
    t1746_im /= 2.0;
    t1746_re /= 2.0;
  }

  c_gradient[777].re = (ct->f136 + ct_re) + t1746_im;
  c_gradient[777].im = ct_im + t1746_re;
  t1648_re = ct->f640 * ct->f666;
  t1649_im = t1648_re * ct->f232.re;
  t1650_re = t1648_re * ct->f232.im;
  t1648_re = ct->f700.re - ct->f736.re;
  t1650_im = ct->f700.im - ct->f736.im;
  ct_re = t1649_im * t1648_re - t1650_re * t1650_im;
  ct_im = t1649_im * t1650_im + t1650_re * t1648_re;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1651_re = ct->f649 * ct->f666;
  t1651_im = t1651_re * ct->f232.re;
  t1652_re = t1651_re * ct->f232.im;
  t1651_re = t1651_im * t1648_re - t1652_re * t1650_im;
  t1652_im = t1651_im * t1650_im + t1652_re * t1648_re;
  if (t1652_im == 0.0) {
    t1746_im = t1651_re / 2.0;
    t1746_re = 0.0;
  } else if (t1651_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1652_im / 2.0;
  } else {
    t1746_im = t1651_re / 2.0;
    t1746_re = t1652_im / 2.0;
  }

  c_gradient[778].re = (ct->f144 + ct_re) + t1746_im;
  c_gradient[778].im = ct_im + t1746_re;
  t1653_re = ct->f641 * ct->f669;
  t1653_im = t1653_re * ct->f233.re;
  t1654_re = t1653_re * ct->f233.im;
  t1653_re = ct->f703.re - ct->f739.re;
  t1654_im = ct->f703.im - ct->f739.im;
  t1655_re = t1653_im * t1653_re - t1654_re * t1654_im;
  t1655_im = t1653_im * t1654_im + t1654_re * t1653_re;
  if (t1655_im == 0.0) {
    ct_re = t1655_re / 2.0;
    ct_im = 0.0;
  } else if (t1655_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1655_im / 2.0;
  } else {
    ct_re = t1655_re / 2.0;
    ct_im = t1655_im / 2.0;
  }

  t1656_re = ct->f650 * ct->f669;
  t1656_im = t1656_re * ct->f233.re;
  t1657_re = t1656_re * ct->f233.im;
  t1656_re = t1656_im * t1653_re - t1657_re * t1654_im;
  t1657_im = t1656_im * t1654_im + t1657_re * t1653_re;
  if (t1657_im == 0.0) {
    t1746_im = t1656_re / 2.0;
    t1746_re = 0.0;
  } else if (t1656_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1657_im / 2.0;
  } else {
    t1746_im = t1656_re / 2.0;
    t1746_re = t1657_im / 2.0;
  }

  c_gradient[779].re = (ct->f152 + ct_re) + t1746_im;
  c_gradient[779].im = ct_im + t1746_re;
  t1658_re = ct->f642 * ct->f671;
  t1658_im = t1658_re * ct->f234.re;
  t1659_re = t1658_re * ct->f234.im;
  t1658_re = ct->f706.re - ct->f742.re;
  t1659_im = ct->f706.im - ct->f742.im;
  t1660_re = t1658_im * t1658_re - t1659_re * t1659_im;
  t1660_im = t1658_im * t1659_im + t1659_re * t1658_re;
  if (t1660_im == 0.0) {
    ct_re = t1660_re / 2.0;
    ct_im = 0.0;
  } else if (t1660_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1660_im / 2.0;
  } else {
    ct_re = t1660_re / 2.0;
    ct_im = t1660_im / 2.0;
  }

  t1661_re = ct->f651 * ct->f671;
  t1661_im = t1661_re * ct->f234.re;
  t1662_re = t1661_re * ct->f234.im;
  t1661_re = t1661_im * t1658_re - t1662_re * t1659_im;
  t1662_im = t1661_im * t1659_im + t1662_re * t1658_re;
  if (t1662_im == 0.0) {
    t1746_im = t1661_re / 2.0;
    t1746_re = 0.0;
  } else if (t1661_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1662_im / 2.0;
  } else {
    t1746_im = t1661_re / 2.0;
    t1746_re = t1662_im / 2.0;
  }

  c_gradient[780].re = (ct->f160 + ct_re) + t1746_im;
  c_gradient[780].im = ct_im + t1746_re;
  t1663_re = ct->f643 * ct->f673;
  t1663_im = t1663_re * ct->f235.re;
  t1664_re = t1663_re * ct->f235.im;
  t1663_re = ct->f709.re - ct->f745.re;
  t1664_im = ct->f709.im - ct->f745.im;
  t1665_re = t1663_im * t1663_re - t1664_re * t1664_im;
  t1665_im = t1663_im * t1664_im + t1664_re * t1663_re;
  if (t1665_im == 0.0) {
    ct_re = t1665_re / 2.0;
    ct_im = 0.0;
  } else if (t1665_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1665_im / 2.0;
  } else {
    ct_re = t1665_re / 2.0;
    ct_im = t1665_im / 2.0;
  }

  t1666_re = ct->f652 * ct->f673;
  t1666_im = t1666_re * ct->f235.re;
  t1667_re = t1666_re * ct->f235.im;
  t1666_re = t1666_im * t1663_re - t1667_re * t1664_im;
  t1667_im = t1666_im * t1664_im + t1667_re * t1663_re;
  if (t1667_im == 0.0) {
    t1746_im = t1666_re / 2.0;
    t1746_re = 0.0;
  } else if (t1666_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1667_im / 2.0;
  } else {
    t1746_im = t1666_re / 2.0;
    t1746_re = t1667_im / 2.0;
  }

  c_gradient[781].re = (ct->f168 + ct_re) + t1746_im;
  c_gradient[781].im = ct_im + t1746_re;
  t1668_re = ct->f644 * ct->f675;
  t1668_im = t1668_re * ct->f236.re;
  t1669_re = t1668_re * ct->f236.im;
  t1668_re = ct->f712.re - ct->f748.re;
  t1669_im = ct->f712.im - ct->f748.im;
  t1670_re = t1668_im * t1668_re - t1669_re * t1669_im;
  t1670_im = t1668_im * t1669_im + t1669_re * t1668_re;
  if (t1670_im == 0.0) {
    ct_re = t1670_re / 2.0;
    ct_im = 0.0;
  } else if (t1670_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1670_im / 2.0;
  } else {
    ct_re = t1670_re / 2.0;
    ct_im = t1670_im / 2.0;
  }

  t1671_re = ct->f653 * ct->f675;
  t1671_im = t1671_re * ct->f236.re;
  t1672_re = t1671_re * ct->f236.im;
  t1671_re = t1671_im * t1668_re - t1672_re * t1669_im;
  t1672_im = t1671_im * t1669_im + t1672_re * t1668_re;
  if (t1672_im == 0.0) {
    t1746_im = t1671_re / 2.0;
    t1746_re = 0.0;
  } else if (t1671_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1672_im / 2.0;
  } else {
    t1746_im = t1671_re / 2.0;
    t1746_re = t1672_im / 2.0;
  }

  c_gradient[782].re = (ct->f176 + ct_re) + t1746_im;
  c_gradient[782].im = ct_im + t1746_re;
  t1673_re = ct->f645 * ct->f677;
  t1673_im = t1673_re * ct->f237.re;
  t1674_re = t1673_re * ct->f237.im;
  t1673_re = ct->f715.re - ct->f751.re;
  t1674_im = ct->f715.im - ct->f751.im;
  t1675_re = t1673_im * t1673_re - t1674_re * t1674_im;
  t1675_im = t1673_im * t1674_im + t1674_re * t1673_re;
  if (t1675_im == 0.0) {
    ct_re = t1675_re / 2.0;
    ct_im = 0.0;
  } else if (t1675_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1675_im / 2.0;
  } else {
    ct_re = t1675_re / 2.0;
    ct_im = t1675_im / 2.0;
  }

  t1676_re = ct->f654 * ct->f677;
  t1676_im = t1676_re * ct->f237.re;
  t1677_re = t1676_re * ct->f237.im;
  t1676_re = t1676_im * t1673_re - t1677_re * t1674_im;
  t1677_im = t1676_im * t1674_im + t1677_re * t1673_re;
  if (t1677_im == 0.0) {
    t1746_im = t1676_re / 2.0;
    t1746_re = 0.0;
  } else if (t1676_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1677_im / 2.0;
  } else {
    t1746_im = t1676_re / 2.0;
    t1746_re = t1677_im / 2.0;
  }

  c_gradient[783].re = (ct->f181 + ct_re) + t1746_im;
  c_gradient[783].im = ct_im + t1746_re;
  t1678_re = ct->f646 * ct->f680;
  t1678_im = t1678_re * ct->f238.re;
  t1679_re = t1678_re * ct->f238.im;
  t1678_re = ct->f718.re - ct->f754.re;
  t1679_im = ct->f718.im - ct->f754.im;
  t1680_re = t1678_im * t1678_re - t1679_re * t1679_im;
  t1680_im = t1678_im * t1679_im + t1679_re * t1678_re;
  if (t1680_im == 0.0) {
    ct_re = t1680_re / 2.0;
    ct_im = 0.0;
  } else if (t1680_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1680_im / 2.0;
  } else {
    ct_re = t1680_re / 2.0;
    ct_im = t1680_im / 2.0;
  }

  t1681_re = ct->f655 * ct->f680;
  t1681_im = t1681_re * ct->f238.re;
  t1682_re = t1681_re * ct->f238.im;
  t1681_re = t1681_im * t1678_re - t1682_re * t1679_im;
  t1682_im = t1681_im * t1679_im + t1682_re * t1678_re;
  if (t1682_im == 0.0) {
    t1746_im = t1681_re / 2.0;
    t1746_re = 0.0;
  } else if (t1681_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1682_im / 2.0;
  } else {
    t1746_im = t1681_re / 2.0;
    t1746_re = t1682_im / 2.0;
  }

  c_gradient[784].re = (ct->f185 + ct_re) + t1746_im;
  c_gradient[784].im = ct_im + t1746_re;
  t1683_re = ct->f656 * ct->f682;
  t1683_im = t1683_re * ct->f239.re;
  t1684_re = t1683_re * ct->f239.im;
  t1683_re = ct->f721.re - ct->f757.re;
  t1684_im = ct->f721.im - ct->f757.im;
  t1685_re = t1683_im * t1683_re - t1684_re * t1684_im;
  t1685_im = t1683_im * t1684_im + t1684_re * t1683_re;
  if (t1685_im == 0.0) {
    ct_re = t1685_re / 2.0;
    ct_im = 0.0;
  } else if (t1685_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1685_im / 2.0;
  } else {
    ct_re = t1685_re / 2.0;
    ct_im = t1685_im / 2.0;
  }

  t1686_re = ct->f657 * ct->f682;
  t1686_im = t1686_re * ct->f239.re;
  t1687_re = t1686_re * ct->f239.im;
  t1686_re = t1686_im * t1683_re - t1687_re * t1684_im;
  t1687_im = t1686_im * t1684_im + t1687_re * t1683_re;
  if (t1687_im == 0.0) {
    t1746_im = t1686_re / 2.0;
    t1746_re = 0.0;
  } else if (t1686_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1687_im / 2.0;
  } else {
    t1746_im = t1686_re / 2.0;
    t1746_re = t1687_im / 2.0;
  }

  c_gradient[785].re = (ct->f193 + ct_re) + t1746_im;
  c_gradient[785].im = ct_im + t1746_re;
  memset(&c_gradient[786], 0, 21U * sizeof(creal_T));
  c_gradient[807].re = (-ct->f136 + ct->f486.re) + ct->f489.re;
  c_gradient[807].im = ct->f486.im + ct->f489.im;
  c_gradient[808].re = (-ct->f144 + ct->f492.re) + t1651_re * -0.5;
  c_gradient[808].im = ct->f492.im + t1652_im * -0.5;
  c_gradient[809].re = (-ct->f152 + t1655_re * -0.5) + t1656_re * -0.5;
  c_gradient[809].im = t1655_im * -0.5 + t1657_im * -0.5;
  c_gradient[810].re = (-ct->f160 + t1660_re * -0.5) + t1661_re * -0.5;
  c_gradient[810].im = t1660_im * -0.5 + t1662_im * -0.5;
  c_gradient[811].re = (-ct->f168 + t1665_re * -0.5) + t1666_re * -0.5;
  c_gradient[811].im = t1665_im * -0.5 + t1667_im * -0.5;
  c_gradient[812].re = (-ct->f176 + t1670_re * -0.5) + t1671_re * -0.5;
  c_gradient[812].im = t1670_im * -0.5 + t1672_im * -0.5;
  c_gradient[813].re = (-ct->f181 + t1675_re * -0.5) + t1676_re * -0.5;
  c_gradient[813].im = t1675_im * -0.5 + t1677_im * -0.5;
  c_gradient[814].re = (-ct->f185 + t1680_re * -0.5) + t1681_re * -0.5;
  c_gradient[814].im = t1680_im * -0.5 + t1682_im * -0.5;
  c_gradient[815].re = (-ct->f193 + t1685_re * -0.5) + t1686_re * -0.5;
  c_gradient[815].im = t1685_im * -0.5 + t1687_im * -0.5;
  memset(&c_gradient[816], 0, 21U * sizeof(creal_T));
  t1651_re = ct_re_tmp * t1691_re - ct_im_tmp * t1647_im;
  t1652_im = ct_re_tmp * t1647_im + ct_im_tmp * t1691_re;
  if (t1652_im == 0.0) {
    ct_re = t1651_re / 2.0;
    ct_im = 0.0;
  } else if (t1651_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1652_im / 2.0;
  } else {
    ct_re = t1651_re / 2.0;
    ct_im = t1652_im / 2.0;
  }

  t1655_re = e_ct_re_tmp * t1691_re - e_ct_im_tmp * t1647_im;
  t1655_im = e_ct_re_tmp * t1647_im + e_ct_im_tmp * t1691_re;
  if (t1655_im == 0.0) {
    t1746_im = t1655_re / 2.0;
    t1746_re = 0.0;
  } else if (t1655_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1655_im / 2.0;
  } else {
    t1746_im = t1655_re / 2.0;
    t1746_re = t1655_im / 2.0;
  }

  c_gradient[837].re = ((ct->f913 + ct->f139) + ct_re) + t1746_im;
  c_gradient[837].im = ct_im + t1746_re;
  t1656_re = d_ct_re_tmp * t1648_re - d_ct_im_tmp * t1650_im;
  t1657_im = d_ct_re_tmp * t1650_im + d_ct_im_tmp * t1648_re;
  if (t1657_im == 0.0) {
    ct_re = t1656_re / 2.0;
    ct_im = 0.0;
  } else if (t1656_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1657_im / 2.0;
  } else {
    ct_re = t1656_re / 2.0;
    ct_im = t1657_im / 2.0;
  }

  t1660_re = i_ct_re_tmp * t1648_re - i_ct_im_tmp * t1650_im;
  t1660_im = i_ct_re_tmp * t1650_im + i_ct_im_tmp * t1648_re;
  if (t1660_im == 0.0) {
    t1746_im = t1660_re / 2.0;
    t1746_re = 0.0;
  } else if (t1660_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1660_im / 2.0;
  } else {
    t1746_im = t1660_re / 2.0;
    t1746_re = t1660_im / 2.0;
  }

  c_gradient[838].re = ((ct->f919 + ct->f147) + ct_re) + t1746_im;
  c_gradient[838].im = ct_im + t1746_re;
  t1661_re = h_ct_re_tmp * t1653_re - h_ct_im_tmp * t1654_im;
  t1662_im = h_ct_re_tmp * t1654_im + h_ct_im_tmp * t1653_re;
  if (t1662_im == 0.0) {
    ct_re = t1661_re / 2.0;
    ct_im = 0.0;
  } else if (t1661_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1662_im / 2.0;
  } else {
    ct_re = t1661_re / 2.0;
    ct_im = t1662_im / 2.0;
  }

  t1665_re = m_ct_re_tmp * t1653_re - m_ct_im_tmp * t1654_im;
  t1665_im = m_ct_re_tmp * t1654_im + m_ct_im_tmp * t1653_re;
  if (t1665_im == 0.0) {
    t1746_im = t1665_re / 2.0;
    t1746_re = 0.0;
  } else if (t1665_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1665_im / 2.0;
  } else {
    t1746_im = t1665_re / 2.0;
    t1746_re = t1665_im / 2.0;
  }

  c_gradient[839].re = ((ct->f925 + ct->f155) + ct_re) + t1746_im;
  c_gradient[839].im = ct_im + t1746_re;
  t1666_re = l_ct_re_tmp * t1658_re - l_ct_im_tmp * t1659_im;
  t1667_im = l_ct_re_tmp * t1659_im + l_ct_im_tmp * t1658_re;
  if (t1667_im == 0.0) {
    ct_re = t1666_re / 2.0;
    ct_im = 0.0;
  } else if (t1666_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1667_im / 2.0;
  } else {
    ct_re = t1666_re / 2.0;
    ct_im = t1667_im / 2.0;
  }

  t1670_re = q_ct_re_tmp * t1658_re - q_ct_im_tmp * t1659_im;
  t1670_im = q_ct_re_tmp * t1659_im + q_ct_im_tmp * t1658_re;
  if (t1670_im == 0.0) {
    t1746_im = t1670_re / 2.0;
    t1746_re = 0.0;
  } else if (t1670_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1670_im / 2.0;
  } else {
    t1746_im = t1670_re / 2.0;
    t1746_re = t1670_im / 2.0;
  }

  c_gradient[840].re = ((ct->f931 + ct->f163) + ct_re) + t1746_im;
  c_gradient[840].im = ct_im + t1746_re;
  t1671_re = p_ct_re_tmp * t1663_re - p_ct_im_tmp * t1664_im;
  t1672_im = p_ct_re_tmp * t1664_im + p_ct_im_tmp * t1663_re;
  if (t1672_im == 0.0) {
    ct_re = t1671_re / 2.0;
    ct_im = 0.0;
  } else if (t1671_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1672_im / 2.0;
  } else {
    ct_re = t1671_re / 2.0;
    ct_im = t1672_im / 2.0;
  }

  t1675_re = u_ct_re_tmp * t1663_re - u_ct_im_tmp * t1664_im;
  t1675_im = u_ct_re_tmp * t1664_im + u_ct_im_tmp * t1663_re;
  if (t1675_im == 0.0) {
    t1746_im = t1675_re / 2.0;
    t1746_re = 0.0;
  } else if (t1675_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1675_im / 2.0;
  } else {
    t1746_im = t1675_re / 2.0;
    t1746_re = t1675_im / 2.0;
  }

  c_gradient[841].re = ((ct->f937 + ct->f171) + ct_re) + t1746_im;
  c_gradient[841].im = ct_im + t1746_re;
  t1676_re = t_ct_re_tmp * t1668_re - t_ct_im_tmp * t1669_im;
  t1677_im = t_ct_re_tmp * t1669_im + t_ct_im_tmp * t1668_re;
  if (t1677_im == 0.0) {
    ct_re = t1676_re / 2.0;
    ct_im = 0.0;
  } else if (t1676_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1677_im / 2.0;
  } else {
    ct_re = t1676_re / 2.0;
    ct_im = t1677_im / 2.0;
  }

  t1680_re = y_ct_re_tmp * t1668_re - y_ct_im_tmp * t1669_im;
  t1680_im = y_ct_re_tmp * t1669_im + y_ct_im_tmp * t1668_re;
  if (t1680_im == 0.0) {
    t1746_im = t1680_re / 2.0;
    t1746_re = 0.0;
  } else if (t1680_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1680_im / 2.0;
  } else {
    t1746_im = t1680_re / 2.0;
    t1746_re = t1680_im / 2.0;
  }

  c_gradient[842].re = ((ct->f943 + ct->f178) + ct_re) + t1746_im;
  c_gradient[842].im = ct_im + t1746_re;
  t1681_re = x_ct_re_tmp * t1673_re - x_ct_im_tmp * t1674_im;
  t1682_im = x_ct_re_tmp * t1674_im + x_ct_im_tmp * t1673_re;
  if (t1682_im == 0.0) {
    ct_re = t1681_re / 2.0;
    ct_im = 0.0;
  } else if (t1681_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1682_im / 2.0;
  } else {
    ct_re = t1681_re / 2.0;
    ct_im = t1682_im / 2.0;
  }

  t1685_re = db_ct_re_tmp * t1673_re - db_ct_im_tmp * t1674_im;
  t1685_im = db_ct_re_tmp * t1674_im + db_ct_im_tmp * t1673_re;
  if (t1685_im == 0.0) {
    t1746_im = t1685_re / 2.0;
    t1746_re = 0.0;
  } else if (t1685_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1685_im / 2.0;
  } else {
    t1746_im = t1685_re / 2.0;
    t1746_re = t1685_im / 2.0;
  }

  c_gradient[843].re = ((ct->f947 + ct->f182) + ct_re) + t1746_im;
  c_gradient[843].im = ct_im + t1746_re;
  t1686_re = cb_ct_re_tmp * t1678_re - cb_ct_im_tmp * t1679_im;
  t1687_im = cb_ct_re_tmp * t1679_im + cb_ct_im_tmp * t1678_re;
  if (t1687_im == 0.0) {
    ct_re = t1686_re / 2.0;
    ct_im = 0.0;
  } else if (t1686_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1687_im / 2.0;
  } else {
    ct_re = t1686_re / 2.0;
    ct_im = t1687_im / 2.0;
  }

  t1688_re = gb_ct_re_tmp * t1678_re - gb_ct_im_tmp * t1679_im;
  t1688_im = gb_ct_re_tmp * t1679_im + gb_ct_im_tmp * t1678_re;
  if (t1688_im == 0.0) {
    t1746_im = t1688_re / 2.0;
    t1746_re = 0.0;
  } else if (t1688_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1688_im / 2.0;
  } else {
    t1746_im = t1688_re / 2.0;
    t1746_re = t1688_im / 2.0;
  }

  c_gradient[844].re = ((ct->f7 + ct->f186) + ct_re) + t1746_im;
  c_gradient[844].im = ct_im + t1746_re;
  t1689_re = hb_ct_re_tmp * t1683_re - hb_ct_im_tmp * t1684_im;
  t1689_im = hb_ct_re_tmp * t1684_im + hb_ct_im_tmp * t1683_re;
  if (t1689_im == 0.0) {
    ct_re = t1689_re / 2.0;
    ct_im = 0.0;
  } else if (t1689_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1689_im / 2.0;
  } else {
    ct_re = t1689_re / 2.0;
    ct_im = t1689_im / 2.0;
  }

  t1690_re = kb_ct_re_tmp * t1683_re - kb_ct_im_tmp * t1684_im;
  t1690_im = kb_ct_re_tmp * t1684_im + kb_ct_im_tmp * t1683_re;
  if (t1690_im == 0.0) {
    t1746_im = t1690_re / 2.0;
    t1746_re = 0.0;
  } else if (t1690_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1690_im / 2.0;
  } else {
    t1746_im = t1690_re / 2.0;
    t1746_re = t1690_im / 2.0;
  }

  c_gradient[845].re = ((ct->f13 + ct->f194) + ct_re) + t1746_im;
  c_gradient[845].im = ct_im + t1746_re;
  c_gradient[846].re = 0.0;
  c_gradient[846].im = 0.0;
  t1746_im = c_ct_re_tmp * t1691_re - c_ct_im_tmp * t1647_im;
  t1746_re = c_ct_re_tmp * t1647_im + c_ct_im_tmp * t1691_re;
  t1638_re = b_ct_re_tmp * t1691_re - b_ct_im_tmp * t1647_im;
  t1647_im = b_ct_re_tmp * t1647_im + b_ct_im_tmp * t1691_re;
  if (t1647_im == 0.0) {
    ct_re = t1638_re / 2.0;
    ct_im = 0.0;
  } else if (t1638_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1647_im / 2.0;
  } else {
    ct_re = t1638_re / 2.0;
    ct_im = t1647_im / 2.0;
  }

  c_gradient[847].re = ((ct->f34 + ct->f126) + t1746_im * -0.5) + ct_re;
  c_gradient[847].im = t1746_re * -0.5 + ct_im;
  t1691_re = g_ct_re_tmp * t1648_re - g_ct_im_tmp * t1650_im;
  t1638_im = g_ct_re_tmp * t1650_im + g_ct_im_tmp * t1648_re;
  t1639_re = f_ct_re_tmp * t1648_re - f_ct_im_tmp * t1650_im;
  t1650_im = f_ct_re_tmp * t1650_im + f_ct_im_tmp * t1648_re;
  if (t1650_im == 0.0) {
    ct_re = t1639_re / 2.0;
    ct_im = 0.0;
  } else if (t1639_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1650_im / 2.0;
  } else {
    ct_re = t1639_re / 2.0;
    ct_im = t1650_im / 2.0;
  }

  c_gradient[848].re = ((ct->f37 + ct->f130) + t1691_re * -0.5) + ct_re;
  c_gradient[848].im = t1638_im * -0.5 + ct_im;
  t1648_re = k_ct_re_tmp * t1653_re - k_ct_im_tmp * t1654_im;
  t1639_im = k_ct_re_tmp * t1654_im + k_ct_im_tmp * t1653_re;
  t1640_re = j_ct_re_tmp * t1653_re - j_ct_im_tmp * t1654_im;
  t1654_im = j_ct_re_tmp * t1654_im + j_ct_im_tmp * t1653_re;
  if (t1654_im == 0.0) {
    ct_re = t1640_re / 2.0;
    ct_im = 0.0;
  } else if (t1640_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1654_im / 2.0;
  } else {
    ct_re = t1640_re / 2.0;
    ct_im = t1654_im / 2.0;
  }

  c_gradient[849].re = ((ct->f43 + ct->f137) + t1648_re * -0.5) + ct_re;
  c_gradient[849].im = t1639_im * -0.5 + ct_im;
  t1653_re = o_ct_re_tmp * t1658_re - o_ct_im_tmp * t1659_im;
  t1640_im = o_ct_re_tmp * t1659_im + o_ct_im_tmp * t1658_re;
  t1641_re = n_ct_re_tmp * t1658_re - n_ct_im_tmp * t1659_im;
  t1659_im = n_ct_re_tmp * t1659_im + n_ct_im_tmp * t1658_re;
  if (t1659_im == 0.0) {
    ct_re = t1641_re / 2.0;
    ct_im = 0.0;
  } else if (t1641_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1659_im / 2.0;
  } else {
    ct_re = t1641_re / 2.0;
    ct_im = t1659_im / 2.0;
  }

  c_gradient[850].re = ((ct->f49 + ct->f145) + t1653_re * -0.5) + ct_re;
  c_gradient[850].im = t1640_im * -0.5 + ct_im;
  t1658_re = s_ct_re_tmp * t1663_re - s_ct_im_tmp * t1664_im;
  t1641_im = s_ct_re_tmp * t1664_im + s_ct_im_tmp * t1663_re;
  t1642_re = r_ct_re_tmp * t1663_re - r_ct_im_tmp * t1664_im;
  t1664_im = r_ct_re_tmp * t1664_im + r_ct_im_tmp * t1663_re;
  if (t1664_im == 0.0) {
    ct_re = t1642_re / 2.0;
    ct_im = 0.0;
  } else if (t1642_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1664_im / 2.0;
  } else {
    ct_re = t1642_re / 2.0;
    ct_im = t1664_im / 2.0;
  }

  c_gradient[851].re = ((ct->f55 + ct->f153) + t1658_re * -0.5) + ct_re;
  c_gradient[851].im = t1641_im * -0.5 + ct_im;
  t1663_re = w_ct_re_tmp * t1668_re - w_ct_im_tmp * t1669_im;
  t1642_im = w_ct_re_tmp * t1669_im + w_ct_im_tmp * t1668_re;
  t1643_re = v_ct_re_tmp * t1668_re - v_ct_im_tmp * t1669_im;
  t1669_im = v_ct_re_tmp * t1669_im + v_ct_im_tmp * t1668_re;
  if (t1669_im == 0.0) {
    ct_re = t1643_re / 2.0;
    ct_im = 0.0;
  } else if (t1643_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1669_im / 2.0;
  } else {
    ct_re = t1643_re / 2.0;
    ct_im = t1669_im / 2.0;
  }

  c_gradient[852].re = ((ct->f61 + ct->f161) + t1663_re * -0.5) + ct_re;
  c_gradient[852].im = t1642_im * -0.5 + ct_im;
  t1668_re = bb_ct_re_tmp * t1673_re - bb_ct_im_tmp * t1674_im;
  t1643_im = bb_ct_re_tmp * t1674_im + bb_ct_im_tmp * t1673_re;
  t1644_re = ab_ct_re_tmp * t1673_re - ab_ct_im_tmp * t1674_im;
  t1674_im = ab_ct_re_tmp * t1674_im + ab_ct_im_tmp * t1673_re;
  if (t1674_im == 0.0) {
    ct_re = t1644_re / 2.0;
    ct_im = 0.0;
  } else if (t1644_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1674_im / 2.0;
  } else {
    ct_re = t1644_re / 2.0;
    ct_im = t1674_im / 2.0;
  }

  c_gradient[853].re = ((ct->f67 + ct->f169) + t1668_re * -0.5) + ct_re;
  c_gradient[853].im = t1643_im * -0.5 + ct_im;
  t1673_re = fb_ct_re_tmp * t1678_re - fb_ct_im_tmp * t1679_im;
  t1644_im = fb_ct_re_tmp * t1679_im + fb_ct_im_tmp * t1678_re;
  t1645_re = eb_ct_re_tmp * t1678_re - eb_ct_im_tmp * t1679_im;
  t1679_im = eb_ct_re_tmp * t1679_im + eb_ct_im_tmp * t1678_re;
  if (t1679_im == 0.0) {
    ct_re = t1645_re / 2.0;
    ct_im = 0.0;
  } else if (t1645_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1679_im / 2.0;
  } else {
    ct_re = t1645_re / 2.0;
    ct_im = t1679_im / 2.0;
  }

  c_gradient[854].re = ((ct->f73 + ct->f177) + t1673_re * -0.5) + ct_re;
  c_gradient[854].im = t1644_im * -0.5 + ct_im;
  t1678_re = jb_ct_re_tmp * t1683_re - jb_ct_im_tmp * t1684_im;
  t1645_im = jb_ct_re_tmp * t1684_im + jb_ct_im_tmp * t1683_re;
  t1646_re = ib_ct_re_tmp * t1683_re - ib_ct_im_tmp * t1684_im;
  t1684_im = ib_ct_re_tmp * t1684_im + ib_ct_im_tmp * t1683_re;
  if (t1684_im == 0.0) {
    ct_re = t1646_re / 2.0;
    ct_im = 0.0;
  } else if (t1646_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1684_im / 2.0;
  } else {
    ct_re = t1646_re / 2.0;
    ct_im = t1684_im / 2.0;
  }

  c_gradient[855].re = ((ct->f82 + ct->f190) + t1678_re * -0.5) + ct_re;
  c_gradient[855].im = t1645_im * -0.5 + ct_im;
  memset(&c_gradient[856], 0, 11U * sizeof(creal_T));
  c_gradient[867].re = ((ct->f42 + ct->f202) + t1651_re * -0.5) + t1655_re *
    -0.5;
  c_gradient[867].im = t1652_im * -0.5 + t1655_im * -0.5;
  c_gradient[868].re = ((ct->f48 + ct->f206) + t1656_re * -0.5) + t1660_re *
    -0.5;
  c_gradient[868].im = t1657_im * -0.5 + t1660_im * -0.5;
  c_gradient[869].re = ((ct->f54 + ct->f210) + t1661_re * -0.5) + t1665_re *
    -0.5;
  c_gradient[869].im = t1662_im * -0.5 + t1665_im * -0.5;
  c_gradient[870].re = ((ct->f60 + ct->f214) + t1666_re * -0.5) + t1670_re *
    -0.5;
  c_gradient[870].im = t1667_im * -0.5 + t1670_im * -0.5;
  c_gradient[871].re = ((ct->f66 + ct->f218) + t1671_re * -0.5) + t1675_re *
    -0.5;
  c_gradient[871].im = t1672_im * -0.5 + t1675_im * -0.5;
  c_gradient[872].re = ((ct->f72 + ct->f222) + t1676_re * -0.5) + t1680_re *
    -0.5;
  c_gradient[872].im = t1677_im * -0.5 + t1680_im * -0.5;
  c_gradient[873].re = ((ct->f76 + ct->f224) + t1681_re * -0.5) + t1685_re *
    -0.5;
  c_gradient[873].im = t1682_im * -0.5 + t1685_im * -0.5;
  c_gradient[874].re = ((ct->f79 + ct->f226) + t1686_re * -0.5) + t1688_re *
    -0.5;
  c_gradient[874].im = t1687_im * -0.5 + t1688_im * -0.5;
  c_gradient[875].re = ((ct->f85 + ct->f230) + t1689_re * -0.5) + t1690_re *
    -0.5;
  c_gradient[875].im = t1689_im * -0.5 + t1690_im * -0.5;
  c_gradient[876].re = 0.0;
  c_gradient[876].im = 0.0;
  if (t1746_re == 0.0) {
    ct_re = t1746_im / 2.0;
    ct_im = 0.0;
  } else if (t1746_im == 0.0) {
    ct_re = 0.0;
    ct_im = t1746_re / 2.0;
  } else {
    ct_re = t1746_im / 2.0;
    ct_im = t1746_re / 2.0;
  }

  c_gradient[877].re = ((ct->f905 + ct->f196) + t1638_re * -0.5) + ct_re;
  c_gradient[877].im = t1647_im * -0.5 + ct_im;
  if (t1638_im == 0.0) {
    ct_re = t1691_re / 2.0;
    ct_im = 0.0;
  } else if (t1691_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1638_im / 2.0;
  } else {
    ct_re = t1691_re / 2.0;
    ct_im = t1638_im / 2.0;
  }

  c_gradient[878].re = ((ct->f908 + ct->f198) + t1639_re * -0.5) + ct_re;
  c_gradient[878].im = t1650_im * -0.5 + ct_im;
  if (t1639_im == 0.0) {
    ct_re = t1648_re / 2.0;
    ct_im = 0.0;
  } else if (t1648_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1639_im / 2.0;
  } else {
    ct_re = t1648_re / 2.0;
    ct_im = t1639_im / 2.0;
  }

  c_gradient[879].re = ((ct->f914 + ct->f201) + t1640_re * -0.5) + ct_re;
  c_gradient[879].im = t1654_im * -0.5 + ct_im;
  if (t1640_im == 0.0) {
    ct_re = t1653_re / 2.0;
    ct_im = 0.0;
  } else if (t1653_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1640_im / 2.0;
  } else {
    ct_re = t1653_re / 2.0;
    ct_im = t1640_im / 2.0;
  }

  c_gradient[880].re = ((ct->f920 + ct->f205) + t1641_re * -0.5) + ct_re;
  c_gradient[880].im = t1659_im * -0.5 + ct_im;
  if (t1641_im == 0.0) {
    ct_re = t1658_re / 2.0;
    ct_im = 0.0;
  } else if (t1658_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1641_im / 2.0;
  } else {
    ct_re = t1658_re / 2.0;
    ct_im = t1641_im / 2.0;
  }

  c_gradient[881].re = ((ct->f926 + ct->f209) + t1642_re * -0.5) + ct_re;
  c_gradient[881].im = t1664_im * -0.5 + ct_im;
  if (t1642_im == 0.0) {
    ct_re = t1663_re / 2.0;
    ct_im = 0.0;
  } else if (t1663_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1642_im / 2.0;
  } else {
    ct_re = t1663_re / 2.0;
    ct_im = t1642_im / 2.0;
  }

  c_gradient[882].re = ((ct->f932 + ct->f213) + t1643_re * -0.5) + ct_re;
  c_gradient[882].im = t1669_im * -0.5 + ct_im;
  if (t1643_im == 0.0) {
    ct_re = t1668_re / 2.0;
    ct_im = 0.0;
  } else if (t1668_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1643_im / 2.0;
  } else {
    ct_re = t1668_re / 2.0;
    ct_im = t1643_im / 2.0;
  }

  c_gradient[883].re = ((ct->f938 + ct->f217) + t1644_re * -0.5) + ct_re;
  c_gradient[883].im = t1674_im * -0.5 + ct_im;
  if (t1644_im == 0.0) {
    ct_re = t1673_re / 2.0;
    ct_im = 0.0;
  } else if (t1673_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1644_im / 2.0;
  } else {
    ct_re = t1673_re / 2.0;
    ct_im = t1644_im / 2.0;
  }

  c_gradient[884].re = ((ct->f944 + ct->f221) + t1645_re * -0.5) + ct_re;
  c_gradient[884].im = t1679_im * -0.5 + ct_im;
  if (t1645_im == 0.0) {
    ct_re = t1678_re / 2.0;
    ct_im = 0.0;
  } else if (t1678_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1645_im / 2.0;
  } else {
    ct_re = t1678_re / 2.0;
    ct_im = t1645_im / 2.0;
  }

  c_gradient[885].re = ((ct->f10 + ct->f228) + t1646_re * -0.5) + ct_re;
  c_gradient[885].im = t1684_im * -0.5 + ct_im;
  memset(&c_gradient[886], 0, 19U * sizeof(creal_T));
  t1691_re = ct->f695.re - ct->f732.re;
  t1647_im = ct->f695.im - ct->f732.im;
  ct_re = t1646_im * t1691_re - t1647_re * t1647_im;
  ct_im = t1646_im * t1647_im + t1647_re * t1691_re;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1746_im = t1648_im * t1691_re - t1649_re * t1647_im;
  t1746_re = t1648_im * t1647_im + t1649_re * t1691_re;
  if (t1746_re == 0.0) {
    t1746_im /= 2.0;
    t1746_re = 0.0;
  } else if (t1746_im == 0.0) {
    t1746_im = 0.0;
    t1746_re /= 2.0;
  } else {
    t1746_im /= 2.0;
    t1746_re /= 2.0;
  }

  c_gradient[905].re = (ct->f134 + ct_re) + t1746_im;
  c_gradient[905].im = ct_im + t1746_re;
  t1648_re = ct->f699.re - ct->f735.re;
  t1650_im = ct->f699.im - ct->f735.im;
  ct_re = t1649_im * t1648_re - t1650_re * t1650_im;
  ct_im = t1649_im * t1650_im + t1650_re * t1648_re;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1651_re = t1651_im * t1648_re - t1652_re * t1650_im;
  t1652_im = t1651_im * t1650_im + t1652_re * t1648_re;
  if (t1652_im == 0.0) {
    t1746_im = t1651_re / 2.0;
    t1746_re = 0.0;
  } else if (t1651_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1652_im / 2.0;
  } else {
    t1746_im = t1651_re / 2.0;
    t1746_re = t1652_im / 2.0;
  }

  c_gradient[906].re = (ct->f142 + ct_re) + t1746_im;
  c_gradient[906].im = ct_im + t1746_re;
  t1653_re = ct->f702.re - ct->f738.re;
  t1654_im = ct->f702.im - ct->f738.im;
  t1655_re = t1653_im * t1653_re - t1654_re * t1654_im;
  t1655_im = t1653_im * t1654_im + t1654_re * t1653_re;
  if (t1655_im == 0.0) {
    ct_re = t1655_re / 2.0;
    ct_im = 0.0;
  } else if (t1655_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1655_im / 2.0;
  } else {
    ct_re = t1655_re / 2.0;
    ct_im = t1655_im / 2.0;
  }

  t1656_re = t1656_im * t1653_re - t1657_re * t1654_im;
  t1657_im = t1656_im * t1654_im + t1657_re * t1653_re;
  if (t1657_im == 0.0) {
    t1746_im = t1656_re / 2.0;
    t1746_re = 0.0;
  } else if (t1656_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1657_im / 2.0;
  } else {
    t1746_im = t1656_re / 2.0;
    t1746_re = t1657_im / 2.0;
  }

  c_gradient[907].re = (ct->f150 + ct_re) + t1746_im;
  c_gradient[907].im = ct_im + t1746_re;
  t1658_re = ct->f705.re - ct->f741.re;
  t1659_im = ct->f705.im - ct->f741.im;
  t1660_re = t1658_im * t1658_re - t1659_re * t1659_im;
  t1660_im = t1658_im * t1659_im + t1659_re * t1658_re;
  if (t1660_im == 0.0) {
    ct_re = t1660_re / 2.0;
    ct_im = 0.0;
  } else if (t1660_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1660_im / 2.0;
  } else {
    ct_re = t1660_re / 2.0;
    ct_im = t1660_im / 2.0;
  }

  t1661_re = t1661_im * t1658_re - t1662_re * t1659_im;
  t1662_im = t1661_im * t1659_im + t1662_re * t1658_re;
  if (t1662_im == 0.0) {
    t1746_im = t1661_re / 2.0;
    t1746_re = 0.0;
  } else if (t1661_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1662_im / 2.0;
  } else {
    t1746_im = t1661_re / 2.0;
    t1746_re = t1662_im / 2.0;
  }

  c_gradient[908].re = (ct->f158 + ct_re) + t1746_im;
  c_gradient[908].im = ct_im + t1746_re;
  t1663_re = ct->f708.re - ct->f744.re;
  t1664_im = ct->f708.im - ct->f744.im;
  t1665_re = t1663_im * t1663_re - t1664_re * t1664_im;
  t1665_im = t1663_im * t1664_im + t1664_re * t1663_re;
  if (t1665_im == 0.0) {
    ct_re = t1665_re / 2.0;
    ct_im = 0.0;
  } else if (t1665_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1665_im / 2.0;
  } else {
    ct_re = t1665_re / 2.0;
    ct_im = t1665_im / 2.0;
  }

  t1666_re = t1666_im * t1663_re - t1667_re * t1664_im;
  t1667_im = t1666_im * t1664_im + t1667_re * t1663_re;
  if (t1667_im == 0.0) {
    t1746_im = t1666_re / 2.0;
    t1746_re = 0.0;
  } else if (t1666_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1667_im / 2.0;
  } else {
    t1746_im = t1666_re / 2.0;
    t1746_re = t1667_im / 2.0;
  }

  c_gradient[909].re = (ct->f166 + ct_re) + t1746_im;
  c_gradient[909].im = ct_im + t1746_re;
  t1668_re = ct->f711.re - ct->f747.re;
  t1669_im = ct->f711.im - ct->f747.im;
  t1670_re = t1668_im * t1668_re - t1669_re * t1669_im;
  t1670_im = t1668_im * t1669_im + t1669_re * t1668_re;
  if (t1670_im == 0.0) {
    ct_re = t1670_re / 2.0;
    ct_im = 0.0;
  } else if (t1670_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1670_im / 2.0;
  } else {
    ct_re = t1670_re / 2.0;
    ct_im = t1670_im / 2.0;
  }

  t1671_re = t1671_im * t1668_re - t1672_re * t1669_im;
  t1672_im = t1671_im * t1669_im + t1672_re * t1668_re;
  if (t1672_im == 0.0) {
    t1746_im = t1671_re / 2.0;
    t1746_re = 0.0;
  } else if (t1671_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1672_im / 2.0;
  } else {
    t1746_im = t1671_re / 2.0;
    t1746_re = t1672_im / 2.0;
  }

  c_gradient[910].re = (ct->f174 + ct_re) + t1746_im;
  c_gradient[910].im = ct_im + t1746_re;
  t1673_re = ct->f714.re - ct->f750.re;
  t1674_im = ct->f714.im - ct->f750.im;
  t1675_re = t1673_im * t1673_re - t1674_re * t1674_im;
  t1675_im = t1673_im * t1674_im + t1674_re * t1673_re;
  if (t1675_im == 0.0) {
    ct_re = t1675_re / 2.0;
    ct_im = 0.0;
  } else if (t1675_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1675_im / 2.0;
  } else {
    ct_re = t1675_re / 2.0;
    ct_im = t1675_im / 2.0;
  }

  t1676_re = t1676_im * t1673_re - t1677_re * t1674_im;
  t1677_im = t1676_im * t1674_im + t1677_re * t1673_re;
  if (t1677_im == 0.0) {
    t1746_im = t1676_re / 2.0;
    t1746_re = 0.0;
  } else if (t1676_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1677_im / 2.0;
  } else {
    t1746_im = t1676_re / 2.0;
    t1746_re = t1677_im / 2.0;
  }

  c_gradient[911].re = (ct->f179 + ct_re) + t1746_im;
  c_gradient[911].im = ct_im + t1746_re;
  t1678_re = ct->f717.re - ct->f753.re;
  t1679_im = ct->f717.im - ct->f753.im;
  t1680_re = t1678_im * t1678_re - t1679_re * t1679_im;
  t1680_im = t1678_im * t1679_im + t1679_re * t1678_re;
  if (t1680_im == 0.0) {
    ct_re = t1680_re / 2.0;
    ct_im = 0.0;
  } else if (t1680_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1680_im / 2.0;
  } else {
    ct_re = t1680_re / 2.0;
    ct_im = t1680_im / 2.0;
  }

  t1681_re = t1681_im * t1678_re - t1682_re * t1679_im;
  t1682_im = t1681_im * t1679_im + t1682_re * t1678_re;
  if (t1682_im == 0.0) {
    t1746_im = t1681_re / 2.0;
    t1746_re = 0.0;
  } else if (t1681_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1682_im / 2.0;
  } else {
    t1746_im = t1681_re / 2.0;
    t1746_re = t1682_im / 2.0;
  }

  c_gradient[912].re = (ct->f183 + ct_re) + t1746_im;
  c_gradient[912].im = ct_im + t1746_re;
  t1683_re = ct->f720.re - ct->f756.re;
  t1684_im = ct->f720.im - ct->f756.im;
  t1685_re = t1683_im * t1683_re - t1684_re * t1684_im;
  t1685_im = t1683_im * t1684_im + t1684_re * t1683_re;
  if (t1685_im == 0.0) {
    ct_re = t1685_re / 2.0;
    ct_im = 0.0;
  } else if (t1685_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1685_im / 2.0;
  } else {
    ct_re = t1685_re / 2.0;
    ct_im = t1685_im / 2.0;
  }

  t1686_re = t1686_im * t1683_re - t1687_re * t1684_im;
  t1687_im = t1686_im * t1684_im + t1687_re * t1683_re;
  if (t1687_im == 0.0) {
    t1746_im = t1686_re / 2.0;
    t1746_re = 0.0;
  } else if (t1686_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1687_im / 2.0;
  } else {
    t1746_im = t1686_re / 2.0;
    t1746_re = t1687_im / 2.0;
  }

  c_gradient[913].re = (ct->f191 + ct_re) + t1746_im;
  c_gradient[913].im = ct_im + t1746_re;
  memset(&c_gradient[914], 0, 21U * sizeof(creal_T));
  c_gradient[935].re = (-ct->f134 + ct->f485.re) + ct->f488.re;
  c_gradient[935].im = ct->f485.im + ct->f488.im;
  c_gradient[936].re = (-ct->f142 + ct->f491.re) + t1651_re * -0.5;
  c_gradient[936].im = ct->f491.im + t1652_im * -0.5;
  c_gradient[937].re = (-ct->f150 + t1655_re * -0.5) + t1656_re * -0.5;
  c_gradient[937].im = t1655_im * -0.5 + t1657_im * -0.5;
  c_gradient[938].re = (-ct->f158 + t1660_re * -0.5) + t1661_re * -0.5;
  c_gradient[938].im = t1660_im * -0.5 + t1662_im * -0.5;
  c_gradient[939].re = (-ct->f166 + t1665_re * -0.5) + t1666_re * -0.5;
  c_gradient[939].im = t1665_im * -0.5 + t1667_im * -0.5;
  c_gradient[940].re = (-ct->f174 + t1670_re * -0.5) + t1671_re * -0.5;
  c_gradient[940].im = t1670_im * -0.5 + t1672_im * -0.5;
  c_gradient[941].re = (-ct->f179 + t1675_re * -0.5) + t1676_re * -0.5;
  c_gradient[941].im = t1675_im * -0.5 + t1677_im * -0.5;
  c_gradient[942].re = (-ct->f183 + t1680_re * -0.5) + t1681_re * -0.5;
  c_gradient[942].im = t1680_im * -0.5 + t1682_im * -0.5;
  c_gradient[943].re = (-ct->f191 + t1685_re * -0.5) + t1686_re * -0.5;
  c_gradient[943].im = t1685_im * -0.5 + t1687_im * -0.5;
  memset(&c_gradient[944], 0, 21U * sizeof(creal_T));
  t1651_re = ct_re_tmp * t1691_re - ct_im_tmp * t1647_im;
  t1652_im = ct_re_tmp * t1647_im + ct_im_tmp * t1691_re;
  if (t1652_im == 0.0) {
    ct_re = t1651_re / 2.0;
    ct_im = 0.0;
  } else if (t1651_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1652_im / 2.0;
  } else {
    ct_re = t1651_re / 2.0;
    ct_im = t1652_im / 2.0;
  }

  t1655_re = e_ct_re_tmp * t1691_re - e_ct_im_tmp * t1647_im;
  t1655_im = e_ct_re_tmp * t1647_im + e_ct_im_tmp * t1691_re;
  if (t1655_im == 0.0) {
    t1746_im = t1655_re / 2.0;
    t1746_re = 0.0;
  } else if (t1655_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1655_im / 2.0;
  } else {
    t1746_im = t1655_re / 2.0;
    t1746_re = t1655_im / 2.0;
  }

  c_gradient[965].re = ((ct->f911 + ct->f135) + ct_re) + t1746_im;
  c_gradient[965].im = ct_im + t1746_re;
  t1656_re = d_ct_re_tmp * t1648_re - d_ct_im_tmp * t1650_im;
  t1657_im = d_ct_re_tmp * t1650_im + d_ct_im_tmp * t1648_re;
  if (t1657_im == 0.0) {
    ct_re = t1656_re / 2.0;
    ct_im = 0.0;
  } else if (t1656_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1657_im / 2.0;
  } else {
    ct_re = t1656_re / 2.0;
    ct_im = t1657_im / 2.0;
  }

  t1660_re = i_ct_re_tmp * t1648_re - i_ct_im_tmp * t1650_im;
  t1660_im = i_ct_re_tmp * t1650_im + i_ct_im_tmp * t1648_re;
  if (t1660_im == 0.0) {
    t1746_im = t1660_re / 2.0;
    t1746_re = 0.0;
  } else if (t1660_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1660_im / 2.0;
  } else {
    t1746_im = t1660_re / 2.0;
    t1746_re = t1660_im / 2.0;
  }

  c_gradient[966].re = ((ct->f917 + ct->f143) + ct_re) + t1746_im;
  c_gradient[966].im = ct_im + t1746_re;
  t1661_re = h_ct_re_tmp * t1653_re - h_ct_im_tmp * t1654_im;
  t1662_im = h_ct_re_tmp * t1654_im + h_ct_im_tmp * t1653_re;
  if (t1662_im == 0.0) {
    ct_re = t1661_re / 2.0;
    ct_im = 0.0;
  } else if (t1661_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1662_im / 2.0;
  } else {
    ct_re = t1661_re / 2.0;
    ct_im = t1662_im / 2.0;
  }

  t1665_re = m_ct_re_tmp * t1653_re - m_ct_im_tmp * t1654_im;
  t1665_im = m_ct_re_tmp * t1654_im + m_ct_im_tmp * t1653_re;
  if (t1665_im == 0.0) {
    t1746_im = t1665_re / 2.0;
    t1746_re = 0.0;
  } else if (t1665_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1665_im / 2.0;
  } else {
    t1746_im = t1665_re / 2.0;
    t1746_re = t1665_im / 2.0;
  }

  c_gradient[967].re = ((ct->f923 + ct->f151) + ct_re) + t1746_im;
  c_gradient[967].im = ct_im + t1746_re;
  t1666_re = l_ct_re_tmp * t1658_re - l_ct_im_tmp * t1659_im;
  t1667_im = l_ct_re_tmp * t1659_im + l_ct_im_tmp * t1658_re;
  if (t1667_im == 0.0) {
    ct_re = t1666_re / 2.0;
    ct_im = 0.0;
  } else if (t1666_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1667_im / 2.0;
  } else {
    ct_re = t1666_re / 2.0;
    ct_im = t1667_im / 2.0;
  }

  t1670_re = q_ct_re_tmp * t1658_re - q_ct_im_tmp * t1659_im;
  t1670_im = q_ct_re_tmp * t1659_im + q_ct_im_tmp * t1658_re;
  if (t1670_im == 0.0) {
    t1746_im = t1670_re / 2.0;
    t1746_re = 0.0;
  } else if (t1670_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1670_im / 2.0;
  } else {
    t1746_im = t1670_re / 2.0;
    t1746_re = t1670_im / 2.0;
  }

  c_gradient[968].re = ((ct->f929 + ct->f159) + ct_re) + t1746_im;
  c_gradient[968].im = ct_im + t1746_re;
  t1671_re = p_ct_re_tmp * t1663_re - p_ct_im_tmp * t1664_im;
  t1672_im = p_ct_re_tmp * t1664_im + p_ct_im_tmp * t1663_re;
  if (t1672_im == 0.0) {
    ct_re = t1671_re / 2.0;
    ct_im = 0.0;
  } else if (t1671_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1672_im / 2.0;
  } else {
    ct_re = t1671_re / 2.0;
    ct_im = t1672_im / 2.0;
  }

  t1675_re = u_ct_re_tmp * t1663_re - u_ct_im_tmp * t1664_im;
  t1675_im = u_ct_re_tmp * t1664_im + u_ct_im_tmp * t1663_re;
  if (t1675_im == 0.0) {
    t1746_im = t1675_re / 2.0;
    t1746_re = 0.0;
  } else if (t1675_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1675_im / 2.0;
  } else {
    t1746_im = t1675_re / 2.0;
    t1746_re = t1675_im / 2.0;
  }

  c_gradient[969].re = ((ct->f935 + ct->f167) + ct_re) + t1746_im;
  c_gradient[969].im = ct_im + t1746_re;
  t1676_re = t_ct_re_tmp * t1668_re - t_ct_im_tmp * t1669_im;
  t1677_im = t_ct_re_tmp * t1669_im + t_ct_im_tmp * t1668_re;
  if (t1677_im == 0.0) {
    ct_re = t1676_re / 2.0;
    ct_im = 0.0;
  } else if (t1676_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1677_im / 2.0;
  } else {
    ct_re = t1676_re / 2.0;
    ct_im = t1677_im / 2.0;
  }

  t1680_re = y_ct_re_tmp * t1668_re - y_ct_im_tmp * t1669_im;
  t1680_im = y_ct_re_tmp * t1669_im + y_ct_im_tmp * t1668_re;
  if (t1680_im == 0.0) {
    t1746_im = t1680_re / 2.0;
    t1746_re = 0.0;
  } else if (t1680_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1680_im / 2.0;
  } else {
    t1746_im = t1680_re / 2.0;
    t1746_re = t1680_im / 2.0;
  }

  c_gradient[970].re = ((ct->f941 + ct->f175) + ct_re) + t1746_im;
  c_gradient[970].im = ct_im + t1746_re;
  t1681_re = x_ct_re_tmp * t1673_re - x_ct_im_tmp * t1674_im;
  t1682_im = x_ct_re_tmp * t1674_im + x_ct_im_tmp * t1673_re;
  if (t1682_im == 0.0) {
    ct_re = t1681_re / 2.0;
    ct_im = 0.0;
  } else if (t1681_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1682_im / 2.0;
  } else {
    ct_re = t1681_re / 2.0;
    ct_im = t1682_im / 2.0;
  }

  t1685_re = db_ct_re_tmp * t1673_re - db_ct_im_tmp * t1674_im;
  t1685_im = db_ct_re_tmp * t1674_im + db_ct_im_tmp * t1673_re;
  if (t1685_im == 0.0) {
    t1746_im = t1685_re / 2.0;
    t1746_re = 0.0;
  } else if (t1685_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1685_im / 2.0;
  } else {
    t1746_im = t1685_re / 2.0;
    t1746_re = t1685_im / 2.0;
  }

  c_gradient[971].re = ((ct->f946 + ct->f180) + ct_re) + t1746_im;
  c_gradient[971].im = ct_im + t1746_re;
  t1686_re = cb_ct_re_tmp * t1678_re - cb_ct_im_tmp * t1679_im;
  t1687_im = cb_ct_re_tmp * t1679_im + cb_ct_im_tmp * t1678_re;
  if (t1687_im == 0.0) {
    ct_re = t1686_re / 2.0;
    ct_im = 0.0;
  } else if (t1686_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1687_im / 2.0;
  } else {
    ct_re = t1686_re / 2.0;
    ct_im = t1687_im / 2.0;
  }

  t1688_re = gb_ct_re_tmp * t1678_re - gb_ct_im_tmp * t1679_im;
  t1688_im = gb_ct_re_tmp * t1679_im + gb_ct_im_tmp * t1678_re;
  if (t1688_im == 0.0) {
    t1746_im = t1688_re / 2.0;
    t1746_re = 0.0;
  } else if (t1688_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1688_im / 2.0;
  } else {
    t1746_im = t1688_re / 2.0;
    t1746_re = t1688_im / 2.0;
  }

  c_gradient[972].re = ((ct->f6 + ct->f184) + ct_re) + t1746_im;
  c_gradient[972].im = ct_im + t1746_re;
  t1689_re = hb_ct_re_tmp * t1683_re - hb_ct_im_tmp * t1684_im;
  t1689_im = hb_ct_re_tmp * t1684_im + hb_ct_im_tmp * t1683_re;
  if (t1689_im == 0.0) {
    ct_re = t1689_re / 2.0;
    ct_im = 0.0;
  } else if (t1689_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1689_im / 2.0;
  } else {
    ct_re = t1689_re / 2.0;
    ct_im = t1689_im / 2.0;
  }

  t1690_re = kb_ct_re_tmp * t1683_re - kb_ct_im_tmp * t1684_im;
  t1690_im = kb_ct_re_tmp * t1684_im + kb_ct_im_tmp * t1683_re;
  if (t1690_im == 0.0) {
    t1746_im = t1690_re / 2.0;
    t1746_re = 0.0;
  } else if (t1690_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1690_im / 2.0;
  } else {
    t1746_im = t1690_re / 2.0;
    t1746_re = t1690_im / 2.0;
  }

  c_gradient[973].re = ((ct->f12 + ct->f192) + ct_re) + t1746_im;
  c_gradient[973].im = ct_im + t1746_re;
  c_gradient[974].re = 0.0;
  c_gradient[974].im = 0.0;
  t1746_im = c_ct_re_tmp * t1691_re - c_ct_im_tmp * t1647_im;
  t1746_re = c_ct_re_tmp * t1647_im + c_ct_im_tmp * t1691_re;
  t1638_re = b_ct_re_tmp * t1691_re - b_ct_im_tmp * t1647_im;
  t1647_im = b_ct_re_tmp * t1647_im + b_ct_im_tmp * t1691_re;
  if (t1647_im == 0.0) {
    ct_re = t1638_re / 2.0;
    ct_im = 0.0;
  } else if (t1638_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1647_im / 2.0;
  } else {
    ct_re = t1638_re / 2.0;
    ct_im = t1647_im / 2.0;
  }

  c_gradient[975].re = ((ct->f33 + ct->f124) + t1746_im * -0.5) + ct_re;
  c_gradient[975].im = t1746_re * -0.5 + ct_im;
  t1691_re = g_ct_re_tmp * t1648_re - g_ct_im_tmp * t1650_im;
  t1638_im = g_ct_re_tmp * t1650_im + g_ct_im_tmp * t1648_re;
  t1639_re = f_ct_re_tmp * t1648_re - f_ct_im_tmp * t1650_im;
  t1650_im = f_ct_re_tmp * t1650_im + f_ct_im_tmp * t1648_re;
  if (t1650_im == 0.0) {
    ct_re = t1639_re / 2.0;
    ct_im = 0.0;
  } else if (t1639_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1650_im / 2.0;
  } else {
    ct_re = t1639_re / 2.0;
    ct_im = t1650_im / 2.0;
  }

  c_gradient[976].re = ((ct->f36 + ct->f128) + t1691_re * -0.5) + ct_re;
  c_gradient[976].im = t1638_im * -0.5 + ct_im;
  t1648_re = k_ct_re_tmp * t1653_re - k_ct_im_tmp * t1654_im;
  t1639_im = k_ct_re_tmp * t1654_im + k_ct_im_tmp * t1653_re;
  t1640_re = j_ct_re_tmp * t1653_re - j_ct_im_tmp * t1654_im;
  t1654_im = j_ct_re_tmp * t1654_im + j_ct_im_tmp * t1653_re;
  if (t1654_im == 0.0) {
    ct_re = t1640_re / 2.0;
    ct_im = 0.0;
  } else if (t1640_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1654_im / 2.0;
  } else {
    ct_re = t1640_re / 2.0;
    ct_im = t1654_im / 2.0;
  }

  c_gradient[977].re = ((ct->f41 + ct->f132) + t1648_re * -0.5) + ct_re;
  c_gradient[977].im = t1639_im * -0.5 + ct_im;
  t1653_re = o_ct_re_tmp * t1658_re - o_ct_im_tmp * t1659_im;
  t1640_im = o_ct_re_tmp * t1659_im + o_ct_im_tmp * t1658_re;
  t1641_re = n_ct_re_tmp * t1658_re - n_ct_im_tmp * t1659_im;
  t1659_im = n_ct_re_tmp * t1659_im + n_ct_im_tmp * t1658_re;
  if (t1659_im == 0.0) {
    ct_re = t1641_re / 2.0;
    ct_im = 0.0;
  } else if (t1641_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1659_im / 2.0;
  } else {
    ct_re = t1641_re / 2.0;
    ct_im = t1659_im / 2.0;
  }

  c_gradient[978].re = ((ct->f47 + ct->f140) + t1653_re * -0.5) + ct_re;
  c_gradient[978].im = t1640_im * -0.5 + ct_im;
  t1658_re = s_ct_re_tmp * t1663_re - s_ct_im_tmp * t1664_im;
  t1641_im = s_ct_re_tmp * t1664_im + s_ct_im_tmp * t1663_re;
  t1642_re = r_ct_re_tmp * t1663_re - r_ct_im_tmp * t1664_im;
  t1664_im = r_ct_re_tmp * t1664_im + r_ct_im_tmp * t1663_re;
  if (t1664_im == 0.0) {
    ct_re = t1642_re / 2.0;
    ct_im = 0.0;
  } else if (t1642_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1664_im / 2.0;
  } else {
    ct_re = t1642_re / 2.0;
    ct_im = t1664_im / 2.0;
  }

  c_gradient[979].re = ((ct->f53 + ct->f148) + t1658_re * -0.5) + ct_re;
  c_gradient[979].im = t1641_im * -0.5 + ct_im;
  t1663_re = w_ct_re_tmp * t1668_re - w_ct_im_tmp * t1669_im;
  t1642_im = w_ct_re_tmp * t1669_im + w_ct_im_tmp * t1668_re;
  t1643_re = v_ct_re_tmp * t1668_re - v_ct_im_tmp * t1669_im;
  t1669_im = v_ct_re_tmp * t1669_im + v_ct_im_tmp * t1668_re;
  if (t1669_im == 0.0) {
    ct_re = t1643_re / 2.0;
    ct_im = 0.0;
  } else if (t1643_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1669_im / 2.0;
  } else {
    ct_re = t1643_re / 2.0;
    ct_im = t1669_im / 2.0;
  }

  c_gradient[980].re = ((ct->f59 + ct->f156) + t1663_re * -0.5) + ct_re;
  c_gradient[980].im = t1642_im * -0.5 + ct_im;
  t1668_re = bb_ct_re_tmp * t1673_re - bb_ct_im_tmp * t1674_im;
  t1643_im = bb_ct_re_tmp * t1674_im + bb_ct_im_tmp * t1673_re;
  t1644_re = ab_ct_re_tmp * t1673_re - ab_ct_im_tmp * t1674_im;
  t1674_im = ab_ct_re_tmp * t1674_im + ab_ct_im_tmp * t1673_re;
  if (t1674_im == 0.0) {
    ct_re = t1644_re / 2.0;
    ct_im = 0.0;
  } else if (t1644_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1674_im / 2.0;
  } else {
    ct_re = t1644_re / 2.0;
    ct_im = t1674_im / 2.0;
  }

  c_gradient[981].re = ((ct->f65 + ct->f164) + t1668_re * -0.5) + ct_re;
  c_gradient[981].im = t1643_im * -0.5 + ct_im;
  t1673_re = fb_ct_re_tmp * t1678_re - fb_ct_im_tmp * t1679_im;
  t1644_im = fb_ct_re_tmp * t1679_im + fb_ct_im_tmp * t1678_re;
  t1645_re = eb_ct_re_tmp * t1678_re - eb_ct_im_tmp * t1679_im;
  t1679_im = eb_ct_re_tmp * t1679_im + eb_ct_im_tmp * t1678_re;
  if (t1679_im == 0.0) {
    ct_re = t1645_re / 2.0;
    ct_im = 0.0;
  } else if (t1645_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1679_im / 2.0;
  } else {
    ct_re = t1645_re / 2.0;
    ct_im = t1679_im / 2.0;
  }

  c_gradient[982].re = ((ct->f71 + ct->f172) + t1673_re * -0.5) + ct_re;
  c_gradient[982].im = t1644_im * -0.5 + ct_im;
  t1678_re = jb_ct_re_tmp * t1683_re - jb_ct_im_tmp * t1684_im;
  t1645_im = jb_ct_re_tmp * t1684_im + jb_ct_im_tmp * t1683_re;
  t1646_re = ib_ct_re_tmp * t1683_re - ib_ct_im_tmp * t1684_im;
  t1684_im = ib_ct_re_tmp * t1684_im + ib_ct_im_tmp * t1683_re;
  if (t1684_im == 0.0) {
    ct_re = t1646_re / 2.0;
    ct_im = 0.0;
  } else if (t1646_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1684_im / 2.0;
  } else {
    ct_re = t1646_re / 2.0;
    ct_im = t1684_im / 2.0;
  }

  c_gradient[983].re = ((ct->f81 + ct->f188) + t1678_re * -0.5) + ct_re;
  c_gradient[983].im = t1645_im * -0.5 + ct_im;
  memset(&c_gradient[984], 0, 11U * sizeof(creal_T));
  c_gradient[995].re = ((ct->f40 + ct->f200) + t1651_re * -0.5) + t1655_re *
    -0.5;
  c_gradient[995].im = t1652_im * -0.5 + t1655_im * -0.5;
  c_gradient[996].re = ((ct->f46 + ct->f204) + t1656_re * -0.5) + t1660_re *
    -0.5;
  c_gradient[996].im = t1657_im * -0.5 + t1660_im * -0.5;
  c_gradient[997].re = ((ct->f52 + ct->f208) + t1661_re * -0.5) + t1665_re *
    -0.5;
  c_gradient[997].im = t1662_im * -0.5 + t1665_im * -0.5;
  c_gradient[998].re = ((ct->f58 + ct->f212) + t1666_re * -0.5) + t1670_re *
    -0.5;
  c_gradient[998].im = t1667_im * -0.5 + t1670_im * -0.5;
  c_gradient[999].re = ((ct->f64 + ct->f216) + t1671_re * -0.5) + t1675_re *
    -0.5;
  c_gradient[999].im = t1672_im * -0.5 + t1675_im * -0.5;
  c_gradient[1000].re = ((ct->f70 + ct->f220) + t1676_re * -0.5) + t1680_re *
    -0.5;
  c_gradient[1000].im = t1677_im * -0.5 + t1680_im * -0.5;
  c_gradient[1001].re = ((ct->f75 + ct->f223) + t1681_re * -0.5) + t1685_re *
    -0.5;
  c_gradient[1001].im = t1682_im * -0.5 + t1685_im * -0.5;
  c_gradient[1002].re = ((ct->f78 + ct->f225) + t1686_re * -0.5) + t1688_re *
    -0.5;
  c_gradient[1002].im = t1687_im * -0.5 + t1688_im * -0.5;
  c_gradient[1003].re = ((ct->f84 + ct->f229) + t1689_re * -0.5) + t1690_re *
    -0.5;
  c_gradient[1003].im = t1689_im * -0.5 + t1690_im * -0.5;
  c_gradient[1004].re = 0.0;
  c_gradient[1004].im = 0.0;
  if (t1746_re == 0.0) {
    ct_re = t1746_im / 2.0;
    ct_im = 0.0;
  } else if (t1746_im == 0.0) {
    ct_re = 0.0;
    ct_im = t1746_re / 2.0;
  } else {
    ct_re = t1746_im / 2.0;
    ct_im = t1746_re / 2.0;
  }

  c_gradient[1005].re = ((ct->f904 + ct->f195) + t1638_re * -0.5) + ct_re;
  c_gradient[1005].im = t1647_im * -0.5 + ct_im;
  if (t1638_im == 0.0) {
    ct_re = t1691_re / 2.0;
    ct_im = 0.0;
  } else if (t1691_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1638_im / 2.0;
  } else {
    ct_re = t1691_re / 2.0;
    ct_im = t1638_im / 2.0;
  }

  c_gradient[1006].re = ((ct->f907 + ct->f197) + t1639_re * -0.5) + ct_re;
  c_gradient[1006].im = t1650_im * -0.5 + ct_im;
  if (t1639_im == 0.0) {
    ct_re = t1648_re / 2.0;
    ct_im = 0.0;
  } else if (t1648_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1639_im / 2.0;
  } else {
    ct_re = t1648_re / 2.0;
    ct_im = t1639_im / 2.0;
  }

  c_gradient[1007].re = ((ct->f912 + ct->f199) + t1640_re * -0.5) + ct_re;
  c_gradient[1007].im = t1654_im * -0.5 + ct_im;
  if (t1640_im == 0.0) {
    ct_re = t1653_re / 2.0;
    ct_im = 0.0;
  } else if (t1653_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1640_im / 2.0;
  } else {
    ct_re = t1653_re / 2.0;
    ct_im = t1640_im / 2.0;
  }

  c_gradient[1008].re = ((ct->f918 + ct->f203) + t1641_re * -0.5) + ct_re;
  c_gradient[1008].im = t1659_im * -0.5 + ct_im;
  if (t1641_im == 0.0) {
    ct_re = t1658_re / 2.0;
    ct_im = 0.0;
  } else if (t1658_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1641_im / 2.0;
  } else {
    ct_re = t1658_re / 2.0;
    ct_im = t1641_im / 2.0;
  }

  c_gradient[1009].re = ((ct->f924 + ct->f207) + t1642_re * -0.5) + ct_re;
  c_gradient[1009].im = t1664_im * -0.5 + ct_im;
  if (t1642_im == 0.0) {
    ct_re = t1663_re / 2.0;
    ct_im = 0.0;
  } else if (t1663_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1642_im / 2.0;
  } else {
    ct_re = t1663_re / 2.0;
    ct_im = t1642_im / 2.0;
  }

  c_gradient[1010].re = ((ct->f930 + ct->f211) + t1643_re * -0.5) + ct_re;
  c_gradient[1010].im = t1669_im * -0.5 + ct_im;
  if (t1643_im == 0.0) {
    ct_re = t1668_re / 2.0;
    ct_im = 0.0;
  } else if (t1668_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1643_im / 2.0;
  } else {
    ct_re = t1668_re / 2.0;
    ct_im = t1643_im / 2.0;
  }

  c_gradient[1011].re = ((ct->f936 + ct->f215) + t1644_re * -0.5) + ct_re;
  c_gradient[1011].im = t1674_im * -0.5 + ct_im;
  if (t1644_im == 0.0) {
    ct_re = t1673_re / 2.0;
    ct_im = 0.0;
  } else if (t1673_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1644_im / 2.0;
  } else {
    ct_re = t1673_re / 2.0;
    ct_im = t1644_im / 2.0;
  }

  c_gradient[1012].re = ((ct->f942 + ct->f219) + t1645_re * -0.5) + ct_re;
  c_gradient[1012].im = t1679_im * -0.5 + ct_im;
  if (t1645_im == 0.0) {
    ct_re = t1678_re / 2.0;
    ct_im = 0.0;
  } else if (t1678_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1645_im / 2.0;
  } else {
    ct_re = t1678_re / 2.0;
    ct_im = t1645_im / 2.0;
  }

  c_gradient[1013].re = ((ct->f9 + ct->f227) + t1646_re * -0.5) + ct_re;
  c_gradient[1013].im = t1684_im * -0.5 + ct_im;
  memset(&c_gradient[1014], 0, 19U * sizeof(creal_T));
  t1691_re = ct->f694.re - ct->f731.re;
  t1647_im = ct->f694.im - ct->f731.im;
  ct_re = t1646_im * t1691_re - t1647_re * t1647_im;
  ct_im = t1646_im * t1647_im + t1647_re * t1691_re;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1746_im = t1648_im * t1691_re - t1649_re * t1647_im;
  t1746_re = t1648_im * t1647_im + t1649_re * t1691_re;
  if (t1746_re == 0.0) {
    t1746_im /= 2.0;
    t1746_re = 0.0;
  } else if (t1746_im == 0.0) {
    t1746_im = 0.0;
    t1746_re /= 2.0;
  } else {
    t1746_im /= 2.0;
    t1746_re /= 2.0;
  }

  c_gradient[1033].re = (ct->f89 + ct_re) + t1746_im;
  c_gradient[1033].im = ct_im + t1746_re;
  t1648_re = ct->f698.re - ct->f734.re;
  t1650_im = ct->f698.im - ct->f734.im;
  ct_re = t1649_im * t1648_re - t1650_re * t1650_im;
  ct_im = t1649_im * t1650_im + t1650_re * t1648_re;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1746_im = t1651_im * t1648_re - t1652_re * t1650_im;
  t1746_re = t1651_im * t1650_im + t1652_re * t1648_re;
  if (t1746_re == 0.0) {
    t1746_im /= 2.0;
    t1746_re = 0.0;
  } else if (t1746_im == 0.0) {
    t1746_im = 0.0;
    t1746_re /= 2.0;
  } else {
    t1746_im /= 2.0;
    t1746_re /= 2.0;
  }

  c_gradient[1034].re = (ct->f91 + ct_re) + t1746_im;
  c_gradient[1034].im = ct_im + t1746_re;
  t1651_re = ct->f701.re - ct->f737.re;
  t1652_im = ct->f701.im - ct->f737.im;
  t1653_re = t1653_im * t1651_re - t1654_re * t1652_im;
  t1654_im = t1653_im * t1652_im + t1654_re * t1651_re;
  if (t1654_im == 0.0) {
    ct_re = t1653_re / 2.0;
    ct_im = 0.0;
  } else if (t1653_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1654_im / 2.0;
  } else {
    ct_re = t1653_re / 2.0;
    ct_im = t1654_im / 2.0;
  }

  t1655_re = t1656_im * t1651_re - t1657_re * t1652_im;
  t1655_im = t1656_im * t1652_im + t1657_re * t1651_re;
  if (t1655_im == 0.0) {
    t1746_im = t1655_re / 2.0;
    t1746_re = 0.0;
  } else if (t1655_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1655_im / 2.0;
  } else {
    t1746_im = t1655_re / 2.0;
    t1746_re = t1655_im / 2.0;
  }

  c_gradient[1035].re = (ct->f93 + ct_re) + t1746_im;
  c_gradient[1035].im = ct_im + t1746_re;
  t1656_re = ct->f704.re - ct->f740.re;
  t1657_im = ct->f704.im - ct->f740.im;
  t1658_re = t1658_im * t1656_re - t1659_re * t1657_im;
  t1659_im = t1658_im * t1657_im + t1659_re * t1656_re;
  if (t1659_im == 0.0) {
    ct_re = t1658_re / 2.0;
    ct_im = 0.0;
  } else if (t1658_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1659_im / 2.0;
  } else {
    ct_re = t1658_re / 2.0;
    ct_im = t1659_im / 2.0;
  }

  t1660_re = t1661_im * t1656_re - t1662_re * t1657_im;
  t1660_im = t1661_im * t1657_im + t1662_re * t1656_re;
  if (t1660_im == 0.0) {
    t1746_im = t1660_re / 2.0;
    t1746_re = 0.0;
  } else if (t1660_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1660_im / 2.0;
  } else {
    t1746_im = t1660_re / 2.0;
    t1746_re = t1660_im / 2.0;
  }

  c_gradient[1036].re = (ct->f95 + ct_re) + t1746_im;
  c_gradient[1036].im = ct_im + t1746_re;
  t1661_re = ct->f707.re - ct->f743.re;
  t1662_im = ct->f707.im - ct->f743.im;
  t1663_re = t1663_im * t1661_re - t1664_re * t1662_im;
  t1664_im = t1663_im * t1662_im + t1664_re * t1661_re;
  if (t1664_im == 0.0) {
    ct_re = t1663_re / 2.0;
    ct_im = 0.0;
  } else if (t1663_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1664_im / 2.0;
  } else {
    ct_re = t1663_re / 2.0;
    ct_im = t1664_im / 2.0;
  }

  t1665_re = t1666_im * t1661_re - t1667_re * t1662_im;
  t1665_im = t1666_im * t1662_im + t1667_re * t1661_re;
  if (t1665_im == 0.0) {
    t1746_im = t1665_re / 2.0;
    t1746_re = 0.0;
  } else if (t1665_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1665_im / 2.0;
  } else {
    t1746_im = t1665_re / 2.0;
    t1746_re = t1665_im / 2.0;
  }

  c_gradient[1037].re = (ct->f97 + ct_re) + t1746_im;
  c_gradient[1037].im = ct_im + t1746_re;
  t1666_re = ct->f710.re - ct->f746.re;
  t1667_im = ct->f710.im - ct->f746.im;
  t1668_re = t1668_im * t1666_re - t1669_re * t1667_im;
  t1669_im = t1668_im * t1667_im + t1669_re * t1666_re;
  if (t1669_im == 0.0) {
    ct_re = t1668_re / 2.0;
    ct_im = 0.0;
  } else if (t1668_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1669_im / 2.0;
  } else {
    ct_re = t1668_re / 2.0;
    ct_im = t1669_im / 2.0;
  }

  t1670_re = t1671_im * t1666_re - t1672_re * t1667_im;
  t1670_im = t1671_im * t1667_im + t1672_re * t1666_re;
  if (t1670_im == 0.0) {
    t1746_im = t1670_re / 2.0;
    t1746_re = 0.0;
  } else if (t1670_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1670_im / 2.0;
  } else {
    t1746_im = t1670_re / 2.0;
    t1746_re = t1670_im / 2.0;
  }

  c_gradient[1038].re = (ct->f99 + ct_re) + t1746_im;
  c_gradient[1038].im = ct_im + t1746_re;
  t1671_re = ct->f713.re - ct->f749.re;
  t1672_im = ct->f713.im - ct->f749.im;
  t1673_re = t1673_im * t1671_re - t1674_re * t1672_im;
  t1674_im = t1673_im * t1672_im + t1674_re * t1671_re;
  if (t1674_im == 0.0) {
    ct_re = t1673_re / 2.0;
    ct_im = 0.0;
  } else if (t1673_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1674_im / 2.0;
  } else {
    ct_re = t1673_re / 2.0;
    ct_im = t1674_im / 2.0;
  }

  t1675_re = t1676_im * t1671_re - t1677_re * t1672_im;
  t1675_im = t1676_im * t1672_im + t1677_re * t1671_re;
  if (t1675_im == 0.0) {
    t1746_im = t1675_re / 2.0;
    t1746_re = 0.0;
  } else if (t1675_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1675_im / 2.0;
  } else {
    t1746_im = t1675_re / 2.0;
    t1746_re = t1675_im / 2.0;
  }

  c_gradient[1039].re = (ct->f100 + ct_re) + t1746_im;
  c_gradient[1039].im = ct_im + t1746_re;
  t1676_re = ct->f716.re - ct->f752.re;
  t1677_im = ct->f716.im - ct->f752.im;
  t1678_re = t1678_im * t1676_re - t1679_re * t1677_im;
  t1679_im = t1678_im * t1677_im + t1679_re * t1676_re;
  if (t1679_im == 0.0) {
    ct_re = t1678_re / 2.0;
    ct_im = 0.0;
  } else if (t1678_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1679_im / 2.0;
  } else {
    ct_re = t1678_re / 2.0;
    ct_im = t1679_im / 2.0;
  }

  t1680_re = t1681_im * t1676_re - t1682_re * t1677_im;
  t1680_im = t1681_im * t1677_im + t1682_re * t1676_re;
  if (t1680_im == 0.0) {
    t1746_im = t1680_re / 2.0;
    t1746_re = 0.0;
  } else if (t1680_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1680_im / 2.0;
  } else {
    t1746_im = t1680_re / 2.0;
    t1746_re = t1680_im / 2.0;
  }

  c_gradient[1040].re = (ct->f101 + ct_re) + t1746_im;
  c_gradient[1040].im = ct_im + t1746_re;
  t1681_re = ct->f719.re - ct->f755.re;
  t1682_im = ct->f719.im - ct->f755.im;
  t1683_re = t1683_im * t1681_re - t1684_re * t1682_im;
  t1684_im = t1683_im * t1682_im + t1684_re * t1681_re;
  if (t1684_im == 0.0) {
    ct_re = t1683_re / 2.0;
    ct_im = 0.0;
  } else if (t1683_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1684_im / 2.0;
  } else {
    ct_re = t1683_re / 2.0;
    ct_im = t1684_im / 2.0;
  }

  t1685_re = t1686_im * t1681_re - t1687_re * t1682_im;
  t1685_im = t1686_im * t1682_im + t1687_re * t1681_re;
  if (t1685_im == 0.0) {
    t1746_im = t1685_re / 2.0;
    t1746_re = 0.0;
  } else if (t1685_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1685_im / 2.0;
  } else {
    t1746_im = t1685_re / 2.0;
    t1746_re = t1685_im / 2.0;
  }

  c_gradient[1041].re = (ct->f103 + ct_re) + t1746_im;
  c_gradient[1041].im = ct_im + t1746_re;
  memset(&c_gradient[1042], 0, 21U * sizeof(creal_T));
  c_gradient[1063].re = (ct->f483.re + ct->f487.re) - ct->f556 * ct->f628 *
    ct->f663 * 6.0;
  c_gradient[1063].im = ct->f483.im + ct->f487.im;
  c_gradient[1064].re = (ct->f490.re + ct->f493.re) - ct->f561 * ct->f629 *
    ct->f665 * 6.0;
  c_gradient[1064].im = ct->f490.im + ct->f493.im;
  c_gradient[1065].re = (t1653_re * -0.5 + t1655_re * -0.5) - ct->f564 *
    ct->f630 * ct->f667 * 6.0;
  c_gradient[1065].im = t1654_im * -0.5 + t1655_im * -0.5;
  c_gradient[1066].re = (t1658_re * -0.5 + t1660_re * -0.5) - ct->f568 *
    ct->f631 * ct->f670 * 6.0;
  c_gradient[1066].im = t1659_im * -0.5 + t1660_im * -0.5;
  c_gradient[1067].re = (t1663_re * -0.5 + t1665_re * -0.5) - ct->f572 *
    ct->f632 * ct->f672 * 6.0;
  c_gradient[1067].im = t1664_im * -0.5 + t1665_im * -0.5;
  c_gradient[1068].re = (t1668_re * -0.5 + t1670_re * -0.5) - ct->f586 *
    ct->f633 * ct->f674 * 6.0;
  c_gradient[1068].im = t1669_im * -0.5 + t1670_im * -0.5;
  c_gradient[1069].re = (t1673_re * -0.5 + t1675_re * -0.5) - ct->f625 *
    ct->f634 * ct->f676 * 6.0;
  c_gradient[1069].im = t1674_im * -0.5 + t1675_im * -0.5;
  c_gradient[1070].re = (t1678_re * -0.5 + t1680_re * -0.5) - ct->f659 *
    ct->f635 * ct->f678 * 6.0;
  c_gradient[1070].im = t1679_im * -0.5 + t1680_im * -0.5;
  c_gradient[1071].re = (t1683_re * -0.5 + t1685_re * -0.5) - ct->f693 *
    ct->f638 * ct->f681 * 6.0;
  c_gradient[1071].im = t1684_im * -0.5 + t1685_im * -0.5;
  memset(&c_gradient[1072], 0, 21U * sizeof(creal_T));
  t1653_re = ct_re_tmp * t1691_re - ct_im_tmp * t1647_im;
  t1654_im = ct_re_tmp * t1647_im + ct_im_tmp * t1691_re;
  if (t1654_im == 0.0) {
    ct_re = t1653_re / 2.0;
    ct_im = 0.0;
  } else if (t1653_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1654_im / 2.0;
  } else {
    ct_re = t1653_re / 2.0;
    ct_im = t1654_im / 2.0;
  }

  t1655_re = e_ct_re_tmp * t1691_re - e_ct_im_tmp * t1647_im;
  t1655_im = e_ct_re_tmp * t1647_im + e_ct_im_tmp * t1691_re;
  if (t1655_im == 0.0) {
    t1746_im = t1655_re / 2.0;
    t1746_re = 0.0;
  } else if (t1655_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1655_im / 2.0;
  } else {
    t1746_im = t1655_re / 2.0;
    t1746_re = t1655_im / 2.0;
  }

  c_gradient[1093].re = ((ct->f874 + ct->f909) + ct_re) + t1746_im;
  c_gradient[1093].im = ct_im + t1746_re;
  t1658_re = d_ct_re_tmp * t1648_re - d_ct_im_tmp * t1650_im;
  t1659_im = d_ct_re_tmp * t1650_im + d_ct_im_tmp * t1648_re;
  if (t1659_im == 0.0) {
    ct_re = t1658_re / 2.0;
    ct_im = 0.0;
  } else if (t1658_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1659_im / 2.0;
  } else {
    ct_re = t1658_re / 2.0;
    ct_im = t1659_im / 2.0;
  }

  t1660_re = i_ct_re_tmp * t1648_re - i_ct_im_tmp * t1650_im;
  t1660_im = i_ct_re_tmp * t1650_im + i_ct_im_tmp * t1648_re;
  if (t1660_im == 0.0) {
    t1746_im = t1660_re / 2.0;
    t1746_re = 0.0;
  } else if (t1660_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1660_im / 2.0;
  } else {
    t1746_im = t1660_re / 2.0;
    t1746_re = t1660_im / 2.0;
  }

  c_gradient[1094].re = ((ct->f878 + ct->f915) + ct_re) + t1746_im;
  c_gradient[1094].im = ct_im + t1746_re;
  t1663_re = h_ct_re_tmp * t1651_re - h_ct_im_tmp * t1652_im;
  t1664_im = h_ct_re_tmp * t1652_im + h_ct_im_tmp * t1651_re;
  if (t1664_im == 0.0) {
    ct_re = t1663_re / 2.0;
    ct_im = 0.0;
  } else if (t1663_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1664_im / 2.0;
  } else {
    ct_re = t1663_re / 2.0;
    ct_im = t1664_im / 2.0;
  }

  t1665_re = m_ct_re_tmp * t1651_re - m_ct_im_tmp * t1652_im;
  t1665_im = m_ct_re_tmp * t1652_im + m_ct_im_tmp * t1651_re;
  if (t1665_im == 0.0) {
    t1746_im = t1665_re / 2.0;
    t1746_re = 0.0;
  } else if (t1665_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1665_im / 2.0;
  } else {
    t1746_im = t1665_re / 2.0;
    t1746_re = t1665_im / 2.0;
  }

  c_gradient[1095].re = ((ct->f882 + ct->f921) + ct_re) + t1746_im;
  c_gradient[1095].im = ct_im + t1746_re;
  t1668_re = l_ct_re_tmp * t1656_re - l_ct_im_tmp * t1657_im;
  t1669_im = l_ct_re_tmp * t1657_im + l_ct_im_tmp * t1656_re;
  if (t1669_im == 0.0) {
    ct_re = t1668_re / 2.0;
    ct_im = 0.0;
  } else if (t1668_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1669_im / 2.0;
  } else {
    ct_re = t1668_re / 2.0;
    ct_im = t1669_im / 2.0;
  }

  t1670_re = q_ct_re_tmp * t1656_re - q_ct_im_tmp * t1657_im;
  t1670_im = q_ct_re_tmp * t1657_im + q_ct_im_tmp * t1656_re;
  if (t1670_im == 0.0) {
    t1746_im = t1670_re / 2.0;
    t1746_re = 0.0;
  } else if (t1670_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1670_im / 2.0;
  } else {
    t1746_im = t1670_re / 2.0;
    t1746_re = t1670_im / 2.0;
  }

  c_gradient[1096].re = ((ct->f886 + ct->f927) + ct_re) + t1746_im;
  c_gradient[1096].im = ct_im + t1746_re;
  t1673_re = p_ct_re_tmp * t1661_re - p_ct_im_tmp * t1662_im;
  t1674_im = p_ct_re_tmp * t1662_im + p_ct_im_tmp * t1661_re;
  if (t1674_im == 0.0) {
    ct_re = t1673_re / 2.0;
    ct_im = 0.0;
  } else if (t1673_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1674_im / 2.0;
  } else {
    ct_re = t1673_re / 2.0;
    ct_im = t1674_im / 2.0;
  }

  t1675_re = u_ct_re_tmp * t1661_re - u_ct_im_tmp * t1662_im;
  t1675_im = u_ct_re_tmp * t1662_im + u_ct_im_tmp * t1661_re;
  if (t1675_im == 0.0) {
    t1746_im = t1675_re / 2.0;
    t1746_re = 0.0;
  } else if (t1675_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1675_im / 2.0;
  } else {
    t1746_im = t1675_re / 2.0;
    t1746_re = t1675_im / 2.0;
  }

  c_gradient[1097].re = ((ct->f890 + ct->f933) + ct_re) + t1746_im;
  c_gradient[1097].im = ct_im + t1746_re;
  t1678_re = t_ct_re_tmp * t1666_re - t_ct_im_tmp * t1667_im;
  t1679_im = t_ct_re_tmp * t1667_im + t_ct_im_tmp * t1666_re;
  if (t1679_im == 0.0) {
    ct_re = t1678_re / 2.0;
    ct_im = 0.0;
  } else if (t1678_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1679_im / 2.0;
  } else {
    ct_re = t1678_re / 2.0;
    ct_im = t1679_im / 2.0;
  }

  t1680_re = y_ct_re_tmp * t1666_re - y_ct_im_tmp * t1667_im;
  t1680_im = y_ct_re_tmp * t1667_im + y_ct_im_tmp * t1666_re;
  if (t1680_im == 0.0) {
    t1746_im = t1680_re / 2.0;
    t1746_re = 0.0;
  } else if (t1680_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1680_im / 2.0;
  } else {
    t1746_im = t1680_re / 2.0;
    t1746_re = t1680_im / 2.0;
  }

  c_gradient[1098].re = ((ct->f894 + ct->f939) + ct_re) + t1746_im;
  c_gradient[1098].im = ct_im + t1746_re;
  t1683_re = x_ct_re_tmp * t1671_re - x_ct_im_tmp * t1672_im;
  t1684_im = x_ct_re_tmp * t1672_im + x_ct_im_tmp * t1671_re;
  if (t1684_im == 0.0) {
    ct_re = t1683_re / 2.0;
    ct_im = 0.0;
  } else if (t1683_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1684_im / 2.0;
  } else {
    ct_re = t1683_re / 2.0;
    ct_im = t1684_im / 2.0;
  }

  t1685_re = db_ct_re_tmp * t1671_re - db_ct_im_tmp * t1672_im;
  t1685_im = db_ct_re_tmp * t1672_im + db_ct_im_tmp * t1671_re;
  if (t1685_im == 0.0) {
    t1746_im = t1685_re / 2.0;
    t1746_re = 0.0;
  } else if (t1685_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1685_im / 2.0;
  } else {
    t1746_im = t1685_re / 2.0;
    t1746_re = t1685_im / 2.0;
  }

  c_gradient[1099].re = ((ct->f896 + ct->f945) + ct_re) + t1746_im;
  c_gradient[1099].im = ct_im + t1746_re;
  t1686_re = cb_ct_re_tmp * t1676_re - cb_ct_im_tmp * t1677_im;
  t1687_im = cb_ct_re_tmp * t1677_im + cb_ct_im_tmp * t1676_re;
  if (t1687_im == 0.0) {
    ct_re = t1686_re / 2.0;
    ct_im = 0.0;
  } else if (t1686_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1687_im / 2.0;
  } else {
    ct_re = t1686_re / 2.0;
    ct_im = t1687_im / 2.0;
  }

  t1688_re = gb_ct_re_tmp * t1676_re - gb_ct_im_tmp * t1677_im;
  t1688_im = gb_ct_re_tmp * t1677_im + gb_ct_im_tmp * t1676_re;
  if (t1688_im == 0.0) {
    t1746_im = t1688_re / 2.0;
    t1746_re = 0.0;
  } else if (t1688_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1688_im / 2.0;
  } else {
    t1746_im = t1688_re / 2.0;
    t1746_re = t1688_im / 2.0;
  }

  c_gradient[1100].re = ((ct->f898 + ct->f5) + ct_re) + t1746_im;
  c_gradient[1100].im = ct_im + t1746_re;
  t1689_re = hb_ct_re_tmp * t1681_re - hb_ct_im_tmp * t1682_im;
  t1689_im = hb_ct_re_tmp * t1682_im + hb_ct_im_tmp * t1681_re;
  if (t1689_im == 0.0) {
    ct_re = t1689_re / 2.0;
    ct_im = 0.0;
  } else if (t1689_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1689_im / 2.0;
  } else {
    ct_re = t1689_re / 2.0;
    ct_im = t1689_im / 2.0;
  }

  t1690_re = kb_ct_re_tmp * t1681_re - kb_ct_im_tmp * t1682_im;
  t1690_im = kb_ct_re_tmp * t1682_im + kb_ct_im_tmp * t1681_re;
  if (t1690_im == 0.0) {
    t1746_im = t1690_re / 2.0;
    t1746_re = 0.0;
  } else if (t1690_re == 0.0) {
    t1746_im = 0.0;
    t1746_re = t1690_im / 2.0;
  } else {
    t1746_im = t1690_re / 2.0;
    t1746_re = t1690_im / 2.0;
  }

  c_gradient[1101].re = ((ct->f902 + ct->f11) + ct_re) + t1746_im;
  c_gradient[1101].im = ct_im + t1746_re;
  c_gradient[1102].re = 0.0;
  c_gradient[1102].im = 0.0;
  t1746_im = c_ct_re_tmp * t1691_re - c_ct_im_tmp * t1647_im;
  t1746_re = c_ct_re_tmp * t1647_im + c_ct_im_tmp * t1691_re;
  t1638_re = b_ct_re_tmp * t1691_re - b_ct_im_tmp * t1647_im;
  t1647_im = b_ct_re_tmp * t1647_im + b_ct_im_tmp * t1691_re;
  if (t1647_im == 0.0) {
    ct_re = t1638_re / 2.0;
    ct_im = 0.0;
  } else if (t1638_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1647_im / 2.0;
  } else {
    ct_re = t1638_re / 2.0;
    ct_im = t1647_im / 2.0;
  }

  c_gradient[1103].re = ((ct->f868 + ct->f32) + t1746_im * -0.5) + ct_re;
  c_gradient[1103].im = t1746_re * -0.5 + ct_im;
  t1691_re = g_ct_re_tmp * t1648_re - g_ct_im_tmp * t1650_im;
  t1638_im = g_ct_re_tmp * t1650_im + g_ct_im_tmp * t1648_re;
  t1639_re = f_ct_re_tmp * t1648_re - f_ct_im_tmp * t1650_im;
  t1650_im = f_ct_re_tmp * t1650_im + f_ct_im_tmp * t1648_re;
  if (t1650_im == 0.0) {
    ct_re = t1639_re / 2.0;
    ct_im = 0.0;
  } else if (t1639_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1650_im / 2.0;
  } else {
    ct_re = t1639_re / 2.0;
    ct_im = t1650_im / 2.0;
  }

  c_gradient[1104].re = ((ct->f870 + ct->f35) + t1691_re * -0.5) + ct_re;
  c_gradient[1104].im = t1638_im * -0.5 + ct_im;
  t1648_re = k_ct_re_tmp * t1651_re - k_ct_im_tmp * t1652_im;
  t1639_im = k_ct_re_tmp * t1652_im + k_ct_im_tmp * t1651_re;
  t1640_re = j_ct_re_tmp * t1651_re - j_ct_im_tmp * t1652_im;
  t1652_im = j_ct_re_tmp * t1652_im + j_ct_im_tmp * t1651_re;
  if (t1652_im == 0.0) {
    ct_re = t1640_re / 2.0;
    ct_im = 0.0;
  } else if (t1640_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1652_im / 2.0;
  } else {
    ct_re = t1640_re / 2.0;
    ct_im = t1652_im / 2.0;
  }

  c_gradient[1105].re = ((ct->f872 + ct->f39) + t1648_re * -0.5) + ct_re;
  c_gradient[1105].im = t1639_im * -0.5 + ct_im;
  t1651_re = o_ct_re_tmp * t1656_re - o_ct_im_tmp * t1657_im;
  t1640_im = o_ct_re_tmp * t1657_im + o_ct_im_tmp * t1656_re;
  t1641_re = n_ct_re_tmp * t1656_re - n_ct_im_tmp * t1657_im;
  t1657_im = n_ct_re_tmp * t1657_im + n_ct_im_tmp * t1656_re;
  if (t1657_im == 0.0) {
    ct_re = t1641_re / 2.0;
    ct_im = 0.0;
  } else if (t1641_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1657_im / 2.0;
  } else {
    ct_re = t1641_re / 2.0;
    ct_im = t1657_im / 2.0;
  }

  c_gradient[1106].re = ((ct->f876 + ct->f45) + t1651_re * -0.5) + ct_re;
  c_gradient[1106].im = t1640_im * -0.5 + ct_im;
  t1656_re = s_ct_re_tmp * t1661_re - s_ct_im_tmp * t1662_im;
  t1641_im = s_ct_re_tmp * t1662_im + s_ct_im_tmp * t1661_re;
  t1642_re = r_ct_re_tmp * t1661_re - r_ct_im_tmp * t1662_im;
  t1662_im = r_ct_re_tmp * t1662_im + r_ct_im_tmp * t1661_re;
  if (t1662_im == 0.0) {
    ct_re = t1642_re / 2.0;
    ct_im = 0.0;
  } else if (t1642_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1662_im / 2.0;
  } else {
    ct_re = t1642_re / 2.0;
    ct_im = t1662_im / 2.0;
  }

  c_gradient[1107].re = ((ct->f880 + ct->f51) + t1656_re * -0.5) + ct_re;
  c_gradient[1107].im = t1641_im * -0.5 + ct_im;
  t1661_re = w_ct_re_tmp * t1666_re - w_ct_im_tmp * t1667_im;
  t1642_im = w_ct_re_tmp * t1667_im + w_ct_im_tmp * t1666_re;
  t1643_re = v_ct_re_tmp * t1666_re - v_ct_im_tmp * t1667_im;
  t1667_im = v_ct_re_tmp * t1667_im + v_ct_im_tmp * t1666_re;
  if (t1667_im == 0.0) {
    ct_re = t1643_re / 2.0;
    ct_im = 0.0;
  } else if (t1643_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1667_im / 2.0;
  } else {
    ct_re = t1643_re / 2.0;
    ct_im = t1667_im / 2.0;
  }

  c_gradient[1108].re = ((ct->f884 + ct->f57) + t1661_re * -0.5) + ct_re;
  c_gradient[1108].im = t1642_im * -0.5 + ct_im;
  t1666_re = bb_ct_re_tmp * t1671_re - bb_ct_im_tmp * t1672_im;
  t1643_im = bb_ct_re_tmp * t1672_im + bb_ct_im_tmp * t1671_re;
  t1644_re = ab_ct_re_tmp * t1671_re - ab_ct_im_tmp * t1672_im;
  t1672_im = ab_ct_re_tmp * t1672_im + ab_ct_im_tmp * t1671_re;
  if (t1672_im == 0.0) {
    ct_re = t1644_re / 2.0;
    ct_im = 0.0;
  } else if (t1644_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1672_im / 2.0;
  } else {
    ct_re = t1644_re / 2.0;
    ct_im = t1672_im / 2.0;
  }

  c_gradient[1109].re = ((ct->f888 + ct->f63) + t1666_re * -0.5) + ct_re;
  c_gradient[1109].im = t1643_im * -0.5 + ct_im;
  t1671_re = fb_ct_re_tmp * t1676_re - fb_ct_im_tmp * t1677_im;
  t1644_im = fb_ct_re_tmp * t1677_im + fb_ct_im_tmp * t1676_re;
  t1645_re = eb_ct_re_tmp * t1676_re - eb_ct_im_tmp * t1677_im;
  t1677_im = eb_ct_re_tmp * t1677_im + eb_ct_im_tmp * t1676_re;
  if (t1677_im == 0.0) {
    ct_re = t1645_re / 2.0;
    ct_im = 0.0;
  } else if (t1645_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1677_im / 2.0;
  } else {
    ct_re = t1645_re / 2.0;
    ct_im = t1677_im / 2.0;
  }

  c_gradient[1110].re = ((ct->f892 + ct->f69) + t1671_re * -0.5) + ct_re;
  c_gradient[1110].im = t1644_im * -0.5 + ct_im;
  t1676_re = jb_ct_re_tmp * t1681_re - jb_ct_im_tmp * t1682_im;
  t1645_im = jb_ct_re_tmp * t1682_im + jb_ct_im_tmp * t1681_re;
  t1646_re = ib_ct_re_tmp * t1681_re - ib_ct_im_tmp * t1682_im;
  t1682_im = ib_ct_re_tmp * t1682_im + ib_ct_im_tmp * t1681_re;
  if (t1682_im == 0.0) {
    ct_re = t1646_re / 2.0;
    ct_im = 0.0;
  } else if (t1646_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1682_im / 2.0;
  } else {
    ct_re = t1646_re / 2.0;
    ct_im = t1682_im / 2.0;
  }

  c_gradient[1111].re = ((ct->f900 + ct->f80) + t1676_re * -0.5) + ct_re;
  c_gradient[1111].im = t1645_im * -0.5 + ct_im;
  memset(&c_gradient[1112], 0, 11U * sizeof(creal_T));
  c_gradient[1123].re = ((ct->f17 + ct->f38) + t1653_re * -0.5) + t1655_re *
    -0.5;
  c_gradient[1123].im = t1654_im * -0.5 + t1655_im * -0.5;
  c_gradient[1124].re = ((ct->f19 + ct->f44) + t1658_re * -0.5) + t1660_re *
    -0.5;
  c_gradient[1124].im = t1659_im * -0.5 + t1660_im * -0.5;
  c_gradient[1125].re = ((ct->f21 + ct->f50) + t1663_re * -0.5) + t1665_re *
    -0.5;
  c_gradient[1125].im = t1664_im * -0.5 + t1665_im * -0.5;
  c_gradient[1126].re = ((ct->f23 + ct->f56) + t1668_re * -0.5) + t1670_re *
    -0.5;
  c_gradient[1126].im = t1669_im * -0.5 + t1670_im * -0.5;
  c_gradient[1127].re = ((ct->f25 + ct->f62) + t1673_re * -0.5) + t1675_re *
    -0.5;
  c_gradient[1127].im = t1674_im * -0.5 + t1675_im * -0.5;
  c_gradient[1128].re = ((ct->f27 + ct->f68) + t1678_re * -0.5) + t1680_re *
    -0.5;
  c_gradient[1128].im = t1679_im * -0.5 + t1680_im * -0.5;
  c_gradient[1129].re = ((ct->f28 + ct->f74) + t1683_re * -0.5) + t1685_re *
    -0.5;
  c_gradient[1129].im = t1684_im * -0.5 + t1685_im * -0.5;
  c_gradient[1130].re = ((ct->f29 + ct->f77) + t1686_re * -0.5) + t1688_re *
    -0.5;
  c_gradient[1130].im = t1687_im * -0.5 + t1688_im * -0.5;
  c_gradient[1131].re = ((ct->f31 + ct->f83) + t1689_re * -0.5) + t1690_re *
    -0.5;
  c_gradient[1131].im = t1689_im * -0.5 + t1690_im * -0.5;
  c_gradient[1132].re = 0.0;
  c_gradient[1132].im = 0.0;
  if (t1746_re == 0.0) {
    ct_re = t1746_im / 2.0;
    ct_im = 0.0;
  } else if (t1746_im == 0.0) {
    ct_re = 0.0;
    ct_im = t1746_re / 2.0;
  } else {
    ct_re = t1746_im / 2.0;
    ct_im = t1746_re / 2.0;
  }

  c_gradient[1133].re = ((ct->f903 + ct->f14) + t1638_re * -0.5) + ct_re;
  c_gradient[1133].im = t1647_im * -0.5 + ct_im;
  if (t1638_im == 0.0) {
    ct_re = t1691_re / 2.0;
    ct_im = 0.0;
  } else if (t1691_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1638_im / 2.0;
  } else {
    ct_re = t1691_re / 2.0;
    ct_im = t1638_im / 2.0;
  }

  c_gradient[1134].re = ((ct->f906 + ct->f15) + t1639_re * -0.5) + ct_re;
  c_gradient[1134].im = t1650_im * -0.5 + ct_im;
  if (t1639_im == 0.0) {
    ct_re = t1648_re / 2.0;
    ct_im = 0.0;
  } else if (t1648_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1639_im / 2.0;
  } else {
    ct_re = t1648_re / 2.0;
    ct_im = t1639_im / 2.0;
  }

  c_gradient[1135].re = ((ct->f910 + ct->f16) + t1640_re * -0.5) + ct_re;
  c_gradient[1135].im = t1652_im * -0.5 + ct_im;
  if (t1640_im == 0.0) {
    ct_re = t1651_re / 2.0;
    ct_im = 0.0;
  } else if (t1651_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1640_im / 2.0;
  } else {
    ct_re = t1651_re / 2.0;
    ct_im = t1640_im / 2.0;
  }

  c_gradient[1136].re = ((ct->f916 + ct->f18) + t1641_re * -0.5) + ct_re;
  c_gradient[1136].im = t1657_im * -0.5 + ct_im;
  if (t1641_im == 0.0) {
    ct_re = t1656_re / 2.0;
    ct_im = 0.0;
  } else if (t1656_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1641_im / 2.0;
  } else {
    ct_re = t1656_re / 2.0;
    ct_im = t1641_im / 2.0;
  }

  c_gradient[1137].re = ((ct->f922 + ct->f20) + t1642_re * -0.5) + ct_re;
  c_gradient[1137].im = t1662_im * -0.5 + ct_im;
  if (t1642_im == 0.0) {
    ct_re = t1661_re / 2.0;
    ct_im = 0.0;
  } else if (t1661_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1642_im / 2.0;
  } else {
    ct_re = t1661_re / 2.0;
    ct_im = t1642_im / 2.0;
  }

  c_gradient[1138].re = ((ct->f928 + ct->f22) + t1643_re * -0.5) + ct_re;
  c_gradient[1138].im = t1667_im * -0.5 + ct_im;
  if (t1643_im == 0.0) {
    ct_re = t1666_re / 2.0;
    ct_im = 0.0;
  } else if (t1666_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1643_im / 2.0;
  } else {
    ct_re = t1666_re / 2.0;
    ct_im = t1643_im / 2.0;
  }

  c_gradient[1139].re = ((ct->f934 + ct->f24) + t1644_re * -0.5) + ct_re;
  c_gradient[1139].im = t1672_im * -0.5 + ct_im;
  if (t1644_im == 0.0) {
    ct_re = t1671_re / 2.0;
    ct_im = 0.0;
  } else if (t1671_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1644_im / 2.0;
  } else {
    ct_re = t1671_re / 2.0;
    ct_im = t1644_im / 2.0;
  }

  c_gradient[1140].re = ((ct->f940 + ct->f26) + t1645_re * -0.5) + ct_re;
  c_gradient[1140].im = t1677_im * -0.5 + ct_im;
  if (t1645_im == 0.0) {
    ct_re = t1676_re / 2.0;
    ct_im = 0.0;
  } else if (t1676_re == 0.0) {
    ct_re = 0.0;
    ct_im = t1645_im / 2.0;
  } else {
    ct_re = t1676_re / 2.0;
    ct_im = t1645_im / 2.0;
  }

  c_gradient[1141].re = ((ct->f8 + ct->f30) + t1646_re * -0.5) + ct_re;
  c_gradient[1141].im = t1682_im * -0.5 + ct_im;
  memset(&c_gradient[1142], 0, 19U * sizeof(creal_T));
  t1691_re = ct->f683.re - ct->f722.re;
  t1647_im = ct->f683.im - ct->f722.im;
  ct_re = t1646_im * t1691_re - t1647_re * t1647_im;
  ct_im = t1646_im * t1647_im + t1647_re * t1691_re;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1746_im = t1648_im * t1691_re - t1649_re * t1647_im;
  t1746_re = t1648_im * t1647_im + t1649_re * t1691_re;
  if (t1746_re == 0.0) {
    t1746_im /= 2.0;
    t1746_re = 0.0;
  } else if (t1746_im == 0.0) {
    t1746_im = 0.0;
    t1746_re /= 2.0;
  } else {
    t1746_im /= 2.0;
    t1746_re /= 2.0;
  }

  c_gradient[1161].re = (ct->f873 + ct_re) + t1746_im;
  c_gradient[1161].im = ct_im + t1746_re;
  t1646_im = ct->f684.re - ct->f723.re;
  t1647_re = ct->f684.im - ct->f723.im;
  ct_re = t1649_im * t1646_im - t1650_re * t1647_re;
  ct_im = t1649_im * t1647_re + t1650_re * t1646_im;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1746_im = t1651_im * t1646_im - t1652_re * t1647_re;
  t1746_re = t1651_im * t1647_re + t1652_re * t1646_im;
  if (t1746_re == 0.0) {
    t1746_im /= 2.0;
    t1746_re = 0.0;
  } else if (t1746_im == 0.0) {
    t1746_im = 0.0;
    t1746_re /= 2.0;
  } else {
    t1746_im /= 2.0;
    t1746_re /= 2.0;
  }

  c_gradient[1162].re = (ct->f877 + ct_re) + t1746_im;
  c_gradient[1162].im = ct_im + t1746_re;
  t1648_re = ct->f685.re - ct->f724.re;
  t1649_re = ct->f685.im - ct->f724.im;
  ct_re = t1653_im * t1648_re - t1654_re * t1649_re;
  ct_im = t1653_im * t1649_re + t1654_re * t1648_re;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1746_im = t1656_im * t1648_re - t1657_re * t1649_re;
  t1746_re = t1656_im * t1649_re + t1657_re * t1648_re;
  if (t1746_re == 0.0) {
    t1746_im /= 2.0;
    t1746_re = 0.0;
  } else if (t1746_im == 0.0) {
    t1746_im = 0.0;
    t1746_re /= 2.0;
  } else {
    t1746_im /= 2.0;
    t1746_re /= 2.0;
  }

  c_gradient[1163].re = (ct->f881 + ct_re) + t1746_im;
  c_gradient[1163].im = ct_im + t1746_re;
  t1648_im = ct->f686.re - ct->f725.re;
  t1650_re = ct->f686.im - ct->f725.im;
  ct_re = t1658_im * t1648_im - t1659_re * t1650_re;
  ct_im = t1658_im * t1650_re + t1659_re * t1648_im;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1746_im = t1661_im * t1648_im - t1662_re * t1650_re;
  t1746_re = t1661_im * t1650_re + t1662_re * t1648_im;
  if (t1746_re == 0.0) {
    t1746_im /= 2.0;
    t1746_re = 0.0;
  } else if (t1746_im == 0.0) {
    t1746_im = 0.0;
    t1746_re /= 2.0;
  } else {
    t1746_im /= 2.0;
    t1746_re /= 2.0;
  }

  c_gradient[1164].re = (ct->f885 + ct_re) + t1746_im;
  c_gradient[1164].im = ct_im + t1746_re;
  t1649_im = ct->f687.re - ct->f726.re;
  t1650_im = ct->f687.im - ct->f726.im;
  ct_re = t1663_im * t1649_im - t1664_re * t1650_im;
  ct_im = t1663_im * t1650_im + t1664_re * t1649_im;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1746_im = t1666_im * t1649_im - t1667_re * t1650_im;
  t1746_re = t1666_im * t1650_im + t1667_re * t1649_im;
  if (t1746_re == 0.0) {
    t1746_im /= 2.0;
    t1746_re = 0.0;
  } else if (t1746_im == 0.0) {
    t1746_im = 0.0;
    t1746_re /= 2.0;
  } else {
    t1746_im /= 2.0;
    t1746_re /= 2.0;
  }

  c_gradient[1165].re = (ct->f889 + ct_re) + t1746_im;
  c_gradient[1165].im = ct_im + t1746_re;
  t1651_re = ct->f688.re - ct->f727.re;
  t1652_re = ct->f688.im - ct->f727.im;
  ct_re = t1668_im * t1651_re - t1669_re * t1652_re;
  ct_im = t1668_im * t1652_re + t1669_re * t1651_re;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1746_im = t1671_im * t1651_re - t1672_re * t1652_re;
  t1746_re = t1671_im * t1652_re + t1672_re * t1651_re;
  if (t1746_re == 0.0) {
    t1746_im /= 2.0;
    t1746_re = 0.0;
  } else if (t1746_im == 0.0) {
    t1746_im = 0.0;
    t1746_re /= 2.0;
  } else {
    t1746_im /= 2.0;
    t1746_re /= 2.0;
  }

  c_gradient[1166].re = (ct->f893 + ct_re) + t1746_im;
  c_gradient[1166].im = ct_im + t1746_re;
  t1651_im = ct->f689.re - ct->f728.re;
  t1652_im = ct->f689.im - ct->f728.im;
  ct_re = t1673_im * t1651_im - t1674_re * t1652_im;
  ct_im = t1673_im * t1652_im + t1674_re * t1651_im;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1746_im = t1676_im * t1651_im - t1677_re * t1652_im;
  t1746_re = t1676_im * t1652_im + t1677_re * t1651_im;
  if (t1746_re == 0.0) {
    t1746_im /= 2.0;
    t1746_re = 0.0;
  } else if (t1746_im == 0.0) {
    t1746_im = 0.0;
    t1746_re /= 2.0;
  } else {
    t1746_im /= 2.0;
    t1746_re /= 2.0;
  }

  c_gradient[1167].re = (ct->f895 + ct_re) + t1746_im;
  c_gradient[1167].im = ct_im + t1746_re;
  t1653_re = ct->f691.re - ct->f729.re;
  t1654_re = ct->f691.im - ct->f729.im;
  ct_re = t1678_im * t1653_re - t1679_re * t1654_re;
  ct_im = t1678_im * t1654_re + t1679_re * t1653_re;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1746_im = t1681_im * t1653_re - t1682_re * t1654_re;
  t1746_re = t1681_im * t1654_re + t1682_re * t1653_re;
  if (t1746_re == 0.0) {
    t1746_im /= 2.0;
    t1746_re = 0.0;
  } else if (t1746_im == 0.0) {
    t1746_im = 0.0;
    t1746_re /= 2.0;
  } else {
    t1746_im /= 2.0;
    t1746_re /= 2.0;
  }

  c_gradient[1168].re = (ct->f897 + ct_re) + t1746_im;
  c_gradient[1168].im = ct_im + t1746_re;
  t1653_im = ct->f692.re - ct->f730.re;
  t1654_im = ct->f692.im - ct->f730.im;
  ct_re = t1683_im * t1653_im - t1684_re * t1654_im;
  ct_im = t1683_im * t1654_im + t1684_re * t1653_im;
  if (ct_im == 0.0) {
    ct_re /= 2.0;
    ct_im = 0.0;
  } else if (ct_re == 0.0) {
    ct_re = 0.0;
    ct_im /= 2.0;
  } else {
    ct_re /= 2.0;
    ct_im /= 2.0;
  }

  t1746_im = t1686_im * t1653_im - t1687_re * t1654_im;
  t1746_re = t1686_im * t1654_im + t1687_re * t1653_im;
  if (t1746_re == 0.0) {
    t1746_im /= 2.0;
    t1746_re = 0.0;
  } else if (t1746_im == 0.0) {
    t1746_im = 0.0;
    t1746_re /= 2.0;
  } else {
    t1746_im /= 2.0;
    t1746_re /= 2.0;
  }

  c_gradient[1169].re = (ct->f901 + ct_re) + t1746_im;
  c_gradient[1169].im = ct_im + t1746_re;
  memset(&c_gradient[1170], 0, 21U * sizeof(creal_T));
  c_gradient[1191].re = (-ct->f873 + ct->f464.re) + ct->f465.re;
  c_gradient[1191].im = ct->f464.im + ct->f465.im;
  c_gradient[1192].re = (-ct->f877 + ct->f466.re) + ct->f467.re;
  c_gradient[1192].im = ct->f466.im + ct->f467.im;
  c_gradient[1193].re = (-ct->f881 + ct->f468.re) + ct->f469.re;
  c_gradient[1193].im = ct->f468.im + ct->f469.im;
  c_gradient[1194].re = (-ct->f885 + ct->f470.re) + ct->f471.re;
  c_gradient[1194].im = ct->f470.im + ct->f471.im;
  c_gradient[1195].re = (-ct->f889 + ct->f472.re) + ct->f474.re;
  c_gradient[1195].im = ct->f472.im + ct->f474.im;
  c_gradient[1196].re = (-ct->f893 + ct->f475.re) + ct->f476.re;
  c_gradient[1196].im = ct->f475.im + ct->f476.im;
  c_gradient[1197].re = (-ct->f895 + ct->f477.re) + ct->f478.re;
  c_gradient[1197].im = ct->f477.im + ct->f478.im;
  c_gradient[1198].re = (-ct->f897 + ct->f479.re) + ct->f480.re;
  c_gradient[1198].im = ct->f479.im + ct->f480.im;
  c_gradient[1199].re = (-ct->f901 + ct->f481.re) + ct->f482.re;
  c_gradient[1199].im = ct->f481.im + ct->f482.im;
  memset(&c_gradient[1200], 0, 20U * sizeof(creal_T));
  c_gradient[1220].re = ct->f580;
  c_gradient[1220].im = 0.0;
  t1655_re = ct_re_tmp * t1691_re - ct_im_tmp * t1647_im;
  ct_im_tmp = ct_re_tmp * t1647_im + ct_im_tmp * t1691_re;
  if (ct_im_tmp == 0.0) {
    ct_re = t1655_re / 2.0;
    ct_im = 0.0;
  } else if (t1655_re == 0.0) {
    ct_re = 0.0;
    ct_im = ct_im_tmp / 2.0;
  } else {
    ct_re = t1655_re / 2.0;
    ct_im = ct_im_tmp / 2.0;
  }

  ct_re_tmp = e_ct_re_tmp * t1691_re - e_ct_im_tmp * t1647_im;
  e_ct_im_tmp = e_ct_re_tmp * t1647_im + e_ct_im_tmp * t1691_re;
  if (e_ct_im_tmp == 0.0) {
    t1746_im = ct_re_tmp / 2.0;
    t1746_re = 0.0;
  } else if (ct_re_tmp == 0.0) {
    t1746_im = 0.0;
    t1746_re = e_ct_im_tmp / 2.0;
  } else {
    t1746_im = ct_re_tmp / 2.0;
    t1746_re = e_ct_im_tmp / 2.0;
  }

  c_gradient[1221].re = ((ct->f796 + ct->f815) + ct_re) + t1746_im;
  c_gradient[1221].im = ct_im + t1746_re;
  e_ct_re_tmp = d_ct_re_tmp * t1646_im - d_ct_im_tmp * t1647_re;
  d_ct_im_tmp = d_ct_re_tmp * t1647_re + d_ct_im_tmp * t1646_im;
  if (d_ct_im_tmp == 0.0) {
    ct_re = e_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (e_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = d_ct_im_tmp / 2.0;
  } else {
    ct_re = e_ct_re_tmp / 2.0;
    ct_im = d_ct_im_tmp / 2.0;
  }

  d_ct_re_tmp = i_ct_re_tmp * t1646_im - i_ct_im_tmp * t1647_re;
  i_ct_im_tmp = i_ct_re_tmp * t1647_re + i_ct_im_tmp * t1646_im;
  if (i_ct_im_tmp == 0.0) {
    t1746_im = d_ct_re_tmp / 2.0;
    t1746_re = 0.0;
  } else if (d_ct_re_tmp == 0.0) {
    t1746_im = 0.0;
    t1746_re = i_ct_im_tmp / 2.0;
  } else {
    t1746_im = d_ct_re_tmp / 2.0;
    t1746_re = i_ct_im_tmp / 2.0;
  }

  c_gradient[1222].re = ((ct->f798 + ct->f817) + ct_re) + t1746_im;
  c_gradient[1222].im = ct_im + t1746_re;
  i_ct_re_tmp = h_ct_re_tmp * t1648_re - h_ct_im_tmp * t1649_re;
  h_ct_im_tmp = h_ct_re_tmp * t1649_re + h_ct_im_tmp * t1648_re;
  if (h_ct_im_tmp == 0.0) {
    ct_re = i_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (i_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = h_ct_im_tmp / 2.0;
  } else {
    ct_re = i_ct_re_tmp / 2.0;
    ct_im = h_ct_im_tmp / 2.0;
  }

  h_ct_re_tmp = m_ct_re_tmp * t1648_re - m_ct_im_tmp * t1649_re;
  m_ct_im_tmp = m_ct_re_tmp * t1649_re + m_ct_im_tmp * t1648_re;
  if (m_ct_im_tmp == 0.0) {
    t1746_im = h_ct_re_tmp / 2.0;
    t1746_re = 0.0;
  } else if (h_ct_re_tmp == 0.0) {
    t1746_im = 0.0;
    t1746_re = m_ct_im_tmp / 2.0;
  } else {
    t1746_im = h_ct_re_tmp / 2.0;
    t1746_re = m_ct_im_tmp / 2.0;
  }

  c_gradient[1223].re = ((ct->f800 + ct->f819) + ct_re) + t1746_im;
  c_gradient[1223].im = ct_im + t1746_re;
  m_ct_re_tmp = l_ct_re_tmp * t1648_im - l_ct_im_tmp * t1650_re;
  l_ct_im_tmp = l_ct_re_tmp * t1650_re + l_ct_im_tmp * t1648_im;
  if (l_ct_im_tmp == 0.0) {
    ct_re = m_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (m_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = l_ct_im_tmp / 2.0;
  } else {
    ct_re = m_ct_re_tmp / 2.0;
    ct_im = l_ct_im_tmp / 2.0;
  }

  l_ct_re_tmp = q_ct_re_tmp * t1648_im - q_ct_im_tmp * t1650_re;
  q_ct_im_tmp = q_ct_re_tmp * t1650_re + q_ct_im_tmp * t1648_im;
  if (q_ct_im_tmp == 0.0) {
    t1746_im = l_ct_re_tmp / 2.0;
    t1746_re = 0.0;
  } else if (l_ct_re_tmp == 0.0) {
    t1746_im = 0.0;
    t1746_re = q_ct_im_tmp / 2.0;
  } else {
    t1746_im = l_ct_re_tmp / 2.0;
    t1746_re = q_ct_im_tmp / 2.0;
  }

  c_gradient[1224].re = ((ct->f802 + ct->f821) + ct_re) + t1746_im;
  c_gradient[1224].im = ct_im + t1746_re;
  q_ct_re_tmp = p_ct_re_tmp * t1649_im - p_ct_im_tmp * t1650_im;
  p_ct_im_tmp = p_ct_re_tmp * t1650_im + p_ct_im_tmp * t1649_im;
  if (p_ct_im_tmp == 0.0) {
    ct_re = q_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (q_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = p_ct_im_tmp / 2.0;
  } else {
    ct_re = q_ct_re_tmp / 2.0;
    ct_im = p_ct_im_tmp / 2.0;
  }

  p_ct_re_tmp = u_ct_re_tmp * t1649_im - u_ct_im_tmp * t1650_im;
  u_ct_im_tmp = u_ct_re_tmp * t1650_im + u_ct_im_tmp * t1649_im;
  if (u_ct_im_tmp == 0.0) {
    t1746_im = p_ct_re_tmp / 2.0;
    t1746_re = 0.0;
  } else if (p_ct_re_tmp == 0.0) {
    t1746_im = 0.0;
    t1746_re = u_ct_im_tmp / 2.0;
  } else {
    t1746_im = p_ct_re_tmp / 2.0;
    t1746_re = u_ct_im_tmp / 2.0;
  }

  c_gradient[1225].re = ((ct->f804 + ct->f823) + ct_re) + t1746_im;
  c_gradient[1225].im = ct_im + t1746_re;
  u_ct_re_tmp = t_ct_re_tmp * t1651_re - t_ct_im_tmp * t1652_re;
  t_ct_im_tmp = t_ct_re_tmp * t1652_re + t_ct_im_tmp * t1651_re;
  if (t_ct_im_tmp == 0.0) {
    ct_re = u_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (u_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = t_ct_im_tmp / 2.0;
  } else {
    ct_re = u_ct_re_tmp / 2.0;
    ct_im = t_ct_im_tmp / 2.0;
  }

  t_ct_re_tmp = y_ct_re_tmp * t1651_re - y_ct_im_tmp * t1652_re;
  y_ct_im_tmp = y_ct_re_tmp * t1652_re + y_ct_im_tmp * t1651_re;
  if (y_ct_im_tmp == 0.0) {
    t1746_im = t_ct_re_tmp / 2.0;
    t1746_re = 0.0;
  } else if (t_ct_re_tmp == 0.0) {
    t1746_im = 0.0;
    t1746_re = y_ct_im_tmp / 2.0;
  } else {
    t1746_im = t_ct_re_tmp / 2.0;
    t1746_re = y_ct_im_tmp / 2.0;
  }

  c_gradient[1226].re = ((ct->f806 + ct->f825) + ct_re) + t1746_im;
  c_gradient[1226].im = ct_im + t1746_re;
  y_ct_re_tmp = x_ct_re_tmp * t1651_im - x_ct_im_tmp * t1652_im;
  x_ct_im_tmp = x_ct_re_tmp * t1652_im + x_ct_im_tmp * t1651_im;
  if (x_ct_im_tmp == 0.0) {
    ct_re = y_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (y_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = x_ct_im_tmp / 2.0;
  } else {
    ct_re = y_ct_re_tmp / 2.0;
    ct_im = x_ct_im_tmp / 2.0;
  }

  x_ct_re_tmp = db_ct_re_tmp * t1651_im - db_ct_im_tmp * t1652_im;
  db_ct_im_tmp = db_ct_re_tmp * t1652_im + db_ct_im_tmp * t1651_im;
  if (db_ct_im_tmp == 0.0) {
    t1746_im = x_ct_re_tmp / 2.0;
    t1746_re = 0.0;
  } else if (x_ct_re_tmp == 0.0) {
    t1746_im = 0.0;
    t1746_re = db_ct_im_tmp / 2.0;
  } else {
    t1746_im = x_ct_re_tmp / 2.0;
    t1746_re = db_ct_im_tmp / 2.0;
  }

  c_gradient[1227].re = ((ct->f808 + ct->f826) + ct_re) + t1746_im;
  c_gradient[1227].im = ct_im + t1746_re;
  db_ct_re_tmp = cb_ct_re_tmp * t1653_re - cb_ct_im_tmp * t1654_re;
  cb_ct_im_tmp = cb_ct_re_tmp * t1654_re + cb_ct_im_tmp * t1653_re;
  if (cb_ct_im_tmp == 0.0) {
    ct_re = db_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (db_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = cb_ct_im_tmp / 2.0;
  } else {
    ct_re = db_ct_re_tmp / 2.0;
    ct_im = cb_ct_im_tmp / 2.0;
  }

  cb_ct_re_tmp = gb_ct_re_tmp * t1653_re - gb_ct_im_tmp * t1654_re;
  gb_ct_im_tmp = gb_ct_re_tmp * t1654_re + gb_ct_im_tmp * t1653_re;
  if (gb_ct_im_tmp == 0.0) {
    t1746_im = cb_ct_re_tmp / 2.0;
    t1746_re = 0.0;
  } else if (cb_ct_re_tmp == 0.0) {
    t1746_im = 0.0;
    t1746_re = gb_ct_im_tmp / 2.0;
  } else {
    t1746_im = cb_ct_re_tmp / 2.0;
    t1746_re = gb_ct_im_tmp / 2.0;
  }

  c_gradient[1228].re = ((ct->f809 + ct->f827) + ct_re) + t1746_im;
  c_gradient[1228].im = ct_im + t1746_re;
  gb_ct_re_tmp = hb_ct_re_tmp * t1653_im - hb_ct_im_tmp * t1654_im;
  hb_ct_im_tmp = hb_ct_re_tmp * t1654_im + hb_ct_im_tmp * t1653_im;
  if (hb_ct_im_tmp == 0.0) {
    ct_re = gb_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (gb_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = hb_ct_im_tmp / 2.0;
  } else {
    ct_re = gb_ct_re_tmp / 2.0;
    ct_im = hb_ct_im_tmp / 2.0;
  }

  hb_ct_re_tmp = kb_ct_re_tmp * t1653_im - kb_ct_im_tmp * t1654_im;
  kb_ct_im_tmp = kb_ct_re_tmp * t1654_im + kb_ct_im_tmp * t1653_im;
  if (kb_ct_im_tmp == 0.0) {
    t1746_im = hb_ct_re_tmp / 2.0;
    t1746_re = 0.0;
  } else if (hb_ct_re_tmp == 0.0) {
    t1746_im = 0.0;
    t1746_re = kb_ct_im_tmp / 2.0;
  } else {
    t1746_im = hb_ct_re_tmp / 2.0;
    t1746_re = kb_ct_im_tmp / 2.0;
  }

  c_gradient[1229].re = ((ct->f811 + ct->f829) + ct_re) + t1746_im;
  c_gradient[1229].im = ct_im + t1746_re;
  c_gradient[1230].re = ct->f578;
  c_gradient[1230].im = 0.0;
  kb_ct_re_tmp = c_ct_re_tmp * t1691_re - c_ct_im_tmp * t1647_im;
  c_ct_im_tmp = c_ct_re_tmp * t1647_im + c_ct_im_tmp * t1691_re;
  c_ct_re_tmp = b_ct_re_tmp * t1691_re - b_ct_im_tmp * t1647_im;
  b_ct_im_tmp = b_ct_re_tmp * t1647_im + b_ct_im_tmp * t1691_re;
  if (b_ct_im_tmp == 0.0) {
    ct_re = c_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (c_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = b_ct_im_tmp / 2.0;
  } else {
    ct_re = c_ct_re_tmp / 2.0;
    ct_im = b_ct_im_tmp / 2.0;
  }

  c_gradient[1231].re = ((ct->f812 + ct->f830) + kb_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1231].im = c_ct_im_tmp * -0.5 + ct_im;
  t1691_re = g_ct_re_tmp * t1646_im - g_ct_im_tmp * t1647_re;
  g_ct_im_tmp = g_ct_re_tmp * t1647_re + g_ct_im_tmp * t1646_im;
  b_ct_re_tmp = f_ct_re_tmp * t1646_im - f_ct_im_tmp * t1647_re;
  f_ct_im_tmp = f_ct_re_tmp * t1647_re + f_ct_im_tmp * t1646_im;
  if (f_ct_im_tmp == 0.0) {
    ct_re = b_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (b_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = f_ct_im_tmp / 2.0;
  } else {
    ct_re = b_ct_re_tmp / 2.0;
    ct_im = f_ct_im_tmp / 2.0;
  }

  c_gradient[1232].re = ((ct->f813 + ct->f831) + t1691_re * -0.5) + ct_re;
  c_gradient[1232].im = g_ct_im_tmp * -0.5 + ct_im;
  f_ct_re_tmp = k_ct_re_tmp * t1648_re - k_ct_im_tmp * t1649_re;
  k_ct_im_tmp = k_ct_re_tmp * t1649_re + k_ct_im_tmp * t1648_re;
  g_ct_re_tmp = j_ct_re_tmp * t1648_re - j_ct_im_tmp * t1649_re;
  j_ct_im_tmp = j_ct_re_tmp * t1649_re + j_ct_im_tmp * t1648_re;
  if (j_ct_im_tmp == 0.0) {
    ct_re = g_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (g_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = j_ct_im_tmp / 2.0;
  } else {
    ct_re = g_ct_re_tmp / 2.0;
    ct_im = j_ct_im_tmp / 2.0;
  }

  c_gradient[1233].re = ((ct->f814 + ct->f833) + f_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1233].im = k_ct_im_tmp * -0.5 + ct_im;
  j_ct_re_tmp = o_ct_re_tmp * t1648_im - o_ct_im_tmp * t1650_re;
  o_ct_im_tmp = o_ct_re_tmp * t1650_re + o_ct_im_tmp * t1648_im;
  k_ct_re_tmp = n_ct_re_tmp * t1648_im - n_ct_im_tmp * t1650_re;
  n_ct_im_tmp = n_ct_re_tmp * t1650_re + n_ct_im_tmp * t1648_im;
  if (n_ct_im_tmp == 0.0) {
    ct_re = k_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (k_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = n_ct_im_tmp / 2.0;
  } else {
    ct_re = k_ct_re_tmp / 2.0;
    ct_im = n_ct_im_tmp / 2.0;
  }

  c_gradient[1234].re = ((ct->f816 + ct->f835) + j_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1234].im = o_ct_im_tmp * -0.5 + ct_im;
  n_ct_re_tmp = s_ct_re_tmp * t1649_im - s_ct_im_tmp * t1650_im;
  s_ct_im_tmp = s_ct_re_tmp * t1650_im + s_ct_im_tmp * t1649_im;
  o_ct_re_tmp = r_ct_re_tmp * t1649_im - r_ct_im_tmp * t1650_im;
  r_ct_im_tmp = r_ct_re_tmp * t1650_im + r_ct_im_tmp * t1649_im;
  if (r_ct_im_tmp == 0.0) {
    ct_re = o_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (o_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = r_ct_im_tmp / 2.0;
  } else {
    ct_re = o_ct_re_tmp / 2.0;
    ct_im = r_ct_im_tmp / 2.0;
  }

  c_gradient[1235].re = ((ct->f818 + ct->f837) + n_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1235].im = s_ct_im_tmp * -0.5 + ct_im;
  r_ct_re_tmp = w_ct_re_tmp * t1651_re - w_ct_im_tmp * t1652_re;
  w_ct_im_tmp = w_ct_re_tmp * t1652_re + w_ct_im_tmp * t1651_re;
  s_ct_re_tmp = v_ct_re_tmp * t1651_re - v_ct_im_tmp * t1652_re;
  v_ct_im_tmp = v_ct_re_tmp * t1652_re + v_ct_im_tmp * t1651_re;
  if (v_ct_im_tmp == 0.0) {
    ct_re = s_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (s_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = v_ct_im_tmp / 2.0;
  } else {
    ct_re = s_ct_re_tmp / 2.0;
    ct_im = v_ct_im_tmp / 2.0;
  }

  c_gradient[1236].re = ((ct->f820 + ct->f839) + r_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1236].im = w_ct_im_tmp * -0.5 + ct_im;
  v_ct_re_tmp = bb_ct_re_tmp * t1651_im - bb_ct_im_tmp * t1652_im;
  bb_ct_im_tmp = bb_ct_re_tmp * t1652_im + bb_ct_im_tmp * t1651_im;
  w_ct_re_tmp = ab_ct_re_tmp * t1651_im - ab_ct_im_tmp * t1652_im;
  ab_ct_im_tmp = ab_ct_re_tmp * t1652_im + ab_ct_im_tmp * t1651_im;
  if (ab_ct_im_tmp == 0.0) {
    ct_re = w_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (w_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = ab_ct_im_tmp / 2.0;
  } else {
    ct_re = w_ct_re_tmp / 2.0;
    ct_im = ab_ct_im_tmp / 2.0;
  }

  c_gradient[1237].re = ((ct->f822 + ct->f841) + v_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1237].im = bb_ct_im_tmp * -0.5 + ct_im;
  ab_ct_re_tmp = fb_ct_re_tmp * t1653_re - fb_ct_im_tmp * t1654_re;
  fb_ct_im_tmp = fb_ct_re_tmp * t1654_re + fb_ct_im_tmp * t1653_re;
  bb_ct_re_tmp = eb_ct_re_tmp * t1653_re - eb_ct_im_tmp * t1654_re;
  eb_ct_im_tmp = eb_ct_re_tmp * t1654_re + eb_ct_im_tmp * t1653_re;
  if (eb_ct_im_tmp == 0.0) {
    ct_re = bb_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (bb_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = eb_ct_im_tmp / 2.0;
  } else {
    ct_re = bb_ct_re_tmp / 2.0;
    ct_im = eb_ct_im_tmp / 2.0;
  }

  c_gradient[1238].re = ((ct->f824 + ct->f843) + ab_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1238].im = fb_ct_im_tmp * -0.5 + ct_im;
  eb_ct_re_tmp = jb_ct_re_tmp * t1653_im - jb_ct_im_tmp * t1654_im;
  jb_ct_im_tmp = jb_ct_re_tmp * t1654_im + jb_ct_im_tmp * t1653_im;
  fb_ct_re_tmp = ib_ct_re_tmp * t1653_im - ib_ct_im_tmp * t1654_im;
  ib_ct_im_tmp = ib_ct_re_tmp * t1654_im + ib_ct_im_tmp * t1653_im;
  if (ib_ct_im_tmp == 0.0) {
    ct_re = fb_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (fb_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = ib_ct_im_tmp / 2.0;
  } else {
    ct_re = fb_ct_re_tmp / 2.0;
    ct_im = ib_ct_im_tmp / 2.0;
  }

  c_gradient[1239].re = ((ct->f828 + ct->f846) + eb_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1239].im = jb_ct_im_tmp * -0.5 + ct_im;
  memset(&c_gradient[1240], 0, 10U * sizeof(creal_T));
  c_gradient[1250].re = ct->f584;
  c_gradient[1250].im = 0.0;
  c_gradient[1251].re = ((ct->f832 + ct->f851) + t1655_re * -0.5) + ct_re_tmp *
    -0.5;
  c_gradient[1251].im = ct_im_tmp * -0.5 + e_ct_im_tmp * -0.5;
  c_gradient[1252].re = ((ct->f834 + ct->f853) + e_ct_re_tmp * -0.5) +
    d_ct_re_tmp * -0.5;
  c_gradient[1252].im = d_ct_im_tmp * -0.5 + i_ct_im_tmp * -0.5;
  c_gradient[1253].re = ((ct->f836 + ct->f855) + i_ct_re_tmp * -0.5) +
    h_ct_re_tmp * -0.5;
  c_gradient[1253].im = h_ct_im_tmp * -0.5 + m_ct_im_tmp * -0.5;
  c_gradient[1254].re = ((ct->f838 + ct->f857) + m_ct_re_tmp * -0.5) +
    l_ct_re_tmp * -0.5;
  c_gradient[1254].im = l_ct_im_tmp * -0.5 + q_ct_im_tmp * -0.5;
  c_gradient[1255].re = ((ct->f840 + ct->f859) + q_ct_re_tmp * -0.5) +
    p_ct_re_tmp * -0.5;
  c_gradient[1255].im = p_ct_im_tmp * -0.5 + u_ct_im_tmp * -0.5;
  c_gradient[1256].re = ((ct->f842 + ct->f861) + u_ct_re_tmp * -0.5) +
    t_ct_re_tmp * -0.5;
  c_gradient[1256].im = t_ct_im_tmp * -0.5 + y_ct_im_tmp * -0.5;
  c_gradient[1257].re = ((ct->f844 + ct->f862) + y_ct_re_tmp * -0.5) +
    x_ct_re_tmp * -0.5;
  c_gradient[1257].im = x_ct_im_tmp * -0.5 + db_ct_im_tmp * -0.5;
  c_gradient[1258].re = ((ct->f845 + ct->f863) + db_ct_re_tmp * -0.5) +
    cb_ct_re_tmp * -0.5;
  c_gradient[1258].im = cb_ct_im_tmp * -0.5 + gb_ct_im_tmp * -0.5;
  c_gradient[1259].re = ((ct->f847 + ct->f866) + gb_ct_re_tmp * -0.5) +
    hb_ct_re_tmp * -0.5;
  c_gradient[1259].im = hb_ct_im_tmp * -0.5 + kb_ct_im_tmp * -0.5;
  c_gradient[1260].re = ct->f582;
  c_gradient[1260].im = 0.0;
  if (c_ct_im_tmp == 0.0) {
    ct_re = kb_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (kb_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = c_ct_im_tmp / 2.0;
  } else {
    ct_re = kb_ct_re_tmp / 2.0;
    ct_im = c_ct_im_tmp / 2.0;
  }

  c_gradient[1261].re = ((ct->f794 + ct->f848) + c_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1261].im = b_ct_im_tmp * -0.5 + ct_im;
  if (g_ct_im_tmp == 0.0) {
    ct_re = t1691_re / 2.0;
    ct_im = 0.0;
  } else if (t1691_re == 0.0) {
    ct_re = 0.0;
    ct_im = g_ct_im_tmp / 2.0;
  } else {
    ct_re = t1691_re / 2.0;
    ct_im = g_ct_im_tmp / 2.0;
  }

  c_gradient[1262].re = ((ct->f795 + ct->f849) + b_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1262].im = f_ct_im_tmp * -0.5 + ct_im;
  if (k_ct_im_tmp == 0.0) {
    ct_re = f_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (f_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = k_ct_im_tmp / 2.0;
  } else {
    ct_re = f_ct_re_tmp / 2.0;
    ct_im = k_ct_im_tmp / 2.0;
  }

  c_gradient[1263].re = ((ct->f797 + ct->f850) + g_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1263].im = j_ct_im_tmp * -0.5 + ct_im;
  if (o_ct_im_tmp == 0.0) {
    ct_re = j_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (j_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = o_ct_im_tmp / 2.0;
  } else {
    ct_re = j_ct_re_tmp / 2.0;
    ct_im = o_ct_im_tmp / 2.0;
  }

  c_gradient[1264].re = ((ct->f799 + ct->f852) + k_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1264].im = n_ct_im_tmp * -0.5 + ct_im;
  if (s_ct_im_tmp == 0.0) {
    ct_re = n_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (n_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = s_ct_im_tmp / 2.0;
  } else {
    ct_re = n_ct_re_tmp / 2.0;
    ct_im = s_ct_im_tmp / 2.0;
  }

  c_gradient[1265].re = ((ct->f801 + ct->f854) + o_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1265].im = r_ct_im_tmp * -0.5 + ct_im;
  if (w_ct_im_tmp == 0.0) {
    ct_re = r_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (r_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = w_ct_im_tmp / 2.0;
  } else {
    ct_re = r_ct_re_tmp / 2.0;
    ct_im = w_ct_im_tmp / 2.0;
  }

  c_gradient[1266].re = ((ct->f803 + ct->f856) + s_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1266].im = v_ct_im_tmp * -0.5 + ct_im;
  if (bb_ct_im_tmp == 0.0) {
    ct_re = v_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (v_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = bb_ct_im_tmp / 2.0;
  } else {
    ct_re = v_ct_re_tmp / 2.0;
    ct_im = bb_ct_im_tmp / 2.0;
  }

  c_gradient[1267].re = ((ct->f805 + ct->f858) + w_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1267].im = ab_ct_im_tmp * -0.5 + ct_im;
  if (fb_ct_im_tmp == 0.0) {
    ct_re = ab_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (ab_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = fb_ct_im_tmp / 2.0;
  } else {
    ct_re = ab_ct_re_tmp / 2.0;
    ct_im = fb_ct_im_tmp / 2.0;
  }

  c_gradient[1268].re = ((ct->f807 + ct->f860) + bb_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1268].im = eb_ct_im_tmp * -0.5 + ct_im;
  if (jb_ct_im_tmp == 0.0) {
    ct_re = eb_ct_re_tmp / 2.0;
    ct_im = 0.0;
  } else if (eb_ct_re_tmp == 0.0) {
    ct_re = 0.0;
    ct_im = jb_ct_im_tmp / 2.0;
  } else {
    ct_re = eb_ct_re_tmp / 2.0;
    ct_im = jb_ct_im_tmp / 2.0;
  }

  c_gradient[1269].re = ((ct->f810 + ct->f864) + fb_ct_re_tmp * -0.5) + ct_re;
  c_gradient[1269].im = ib_ct_im_tmp * -0.5 + ct_im;
  memset(&c_gradient[1270], 0, 18U * sizeof(creal_T));
  c_gradient[1288].re = (ct->f580 - ct->f587) - ct->f589;
  c_gradient[1288].im = 0.0;
  c_gradient[1289].re = (ct->f108 - ct->f279.re) - ct->f281.re;
  c_gradient[1289].im = (0.0 - ct->f279.im) - ct->f281.im;
  c_gradient[1290].re = (ct->f110 - ct->f283.re) - ct->f285.re;
  c_gradient[1290].im = (0.0 - ct->f283.im) - ct->f285.im;
  c_gradient[1291].re = (ct->f112 - ct->f287.re) - ct->f289.re;
  c_gradient[1291].im = (0.0 - ct->f287.im) - ct->f289.im;
  c_gradient[1292].re = (ct->f114 - ct->f291.re) - ct->f293.re;
  c_gradient[1292].im = (0.0 - ct->f291.im) - ct->f293.im;
  c_gradient[1293].re = (ct->f116 - ct->f295.re) - ct->f297.re;
  c_gradient[1293].im = (0.0 - ct->f295.im) - ct->f297.im;
  c_gradient[1294].re = (ct->f118 - ct->f299.re) - ct->f301.re;
  c_gradient[1294].im = (0.0 - ct->f299.im) - ct->f301.im;
  c_gradient[1295].re = (ct->f119 - ct->f303.re) - ct->f305.re;
  c_gradient[1295].im = (0.0 - ct->f303.im) - ct->f305.im;
  c_gradient[1296].re = (ct->f120 - ct->f307.re) - ct->f308.re;
  c_gradient[1296].im = (0.0 - ct->f307.im) - ct->f308.im;
  c_gradient[1297].re = (ct->f122 - ct->f311.re) - ct->f312.re;
  c_gradient[1297].im = (0.0 - ct->f311.im) - ct->f312.im;
  memset(&c_gradient[1298], 0, 20U * sizeof(creal_T));
  c_gradient[1318].re = (ct->f584 + ct->f587) + ct->f589;
  c_gradient[1318].im = 0.0;
  c_gradient[1319].re = (-ct->f108 + ct->f279.re) + ct->f281.re;
  c_gradient[1319].im = ct->f279.im + ct->f281.im;
  c_gradient[1320].re = (-ct->f110 + ct->f283.re) + ct->f285.re;
  c_gradient[1320].im = ct->f283.im + ct->f285.im;
  c_gradient[1321].re = (-ct->f112 + ct->f287.re) + ct->f289.re;
  c_gradient[1321].im = ct->f287.im + ct->f289.im;
  c_gradient[1322].re = (-ct->f114 + ct->f291.re) + ct->f293.re;
  c_gradient[1322].im = ct->f291.im + ct->f293.im;
  c_gradient[1323].re = (-ct->f116 + ct->f295.re) + ct->f297.re;
  c_gradient[1323].im = ct->f295.im + ct->f297.im;
  c_gradient[1324].re = (-ct->f118 + ct->f299.re) + ct->f301.re;
  c_gradient[1324].im = ct->f299.im + ct->f301.im;
  c_gradient[1325].re = (-ct->f119 + ct->f303.re) + ct->f305.re;
  c_gradient[1325].im = ct->f303.im + ct->f305.im;
  c_gradient[1326].re = (-ct->f120 + ct->f307.re) + ct->f308.re;
  c_gradient[1326].im = ct->f307.im + ct->f308.im;
  c_gradient[1327].re = (-ct->f122 + ct->f311.re) + ct->f312.re;
  c_gradient[1327].im = ct->f311.im + ct->f312.im;
  memset(&c_gradient[1328], 0, 20U * sizeof(creal_T));
  c_gradient[1348].re = (ct->f579 + ct->f597) - ct->f1 * ct->f105 * ct->f575 *
    2.0;
  c_gradient[1348].im = 0.0;
  c_gradient[1349].re = (ct->f761 - ct->f316.re) - ct->f325.re;
  c_gradient[1349].im = (0.0 - ct->f316.im) - ct->f325.im;
  c_gradient[1350].re = (ct->f763 - ct->f323.re) - ct->f335.re;
  c_gradient[1350].im = (0.0 - ct->f323.im) - ct->f335.im;
  c_gradient[1351].re = (ct->f765 - ct->f333.re) - ct->f343.re;
  c_gradient[1351].im = (0.0 - ct->f333.im) - ct->f343.im;
  c_gradient[1352].re = (ct->f767 - ct->f341.re) - ct->f351.re;
  c_gradient[1352].im = (0.0 - ct->f341.im) - ct->f351.im;
  c_gradient[1353].re = (ct->f769 - ct->f349.re) - ct->f359.re;
  c_gradient[1353].im = (0.0 - ct->f349.im) - ct->f359.im;
  c_gradient[1354].re = (ct->f771 - ct->f357.re) - ct->f367.re;
  c_gradient[1354].im = (0.0 - ct->f357.im) - ct->f367.im;
  c_gradient[1355].re = (ct->f772 - ct->f365.re) - ct->f374.re;
  c_gradient[1355].im = (0.0 - ct->f365.im) - ct->f374.im;
  c_gradient[1356].re = (ct->f773 - ct->f373.re) - ct->f378.re;
  c_gradient[1356].im = (0.0 - ct->f373.im) - ct->f378.im;
  c_gradient[1357].re = (ct->f775 - ct->f382.re) - ct->f386.re;
  c_gradient[1357].im = (0.0 - ct->f382.im) - ct->f386.im;
  c_gradient[1358].re = (ct->f581 + ct->f594) + ct->f598;
  c_gradient[1358].im = 0.0;
  c_gradient[1359].re = (ct->f776 - ct->f319.re) + ct->f321.re;
  c_gradient[1359].im = (0.0 - ct->f319.im) + ct->f321.im;
  c_gradient[1360].re = (ct->f777 - ct->f329.re) + ct->f331.re;
  c_gradient[1360].im = (0.0 - ct->f329.im) + ct->f331.im;
  c_gradient[1361].re = (ct->f778 - ct->f337.re) + ct->f339.re;
  c_gradient[1361].im = (0.0 - ct->f337.im) + ct->f339.im;
  c_gradient[1362].re = (ct->f780 - ct->f345.re) + ct->f347.re;
  c_gradient[1362].im = (0.0 - ct->f345.im) + ct->f347.im;
  c_gradient[1363].re = (ct->f782 - ct->f353.re) + ct->f355.re;
  c_gradient[1363].im = (0.0 - ct->f353.im) + ct->f355.im;
  c_gradient[1364].re = (ct->f784 - ct->f361.re) + ct->f363.re;
  c_gradient[1364].im = (0.0 - ct->f361.im) + ct->f363.im;
  c_gradient[1365].re = (ct->f786 - ct->f369.re) + ct->f371.re;
  c_gradient[1365].im = (0.0 - ct->f369.im) + ct->f371.im;
  c_gradient[1366].re = (ct->f788 - ct->f376.re) + ct->f377.re;
  c_gradient[1366].im = (0.0 - ct->f376.im) + ct->f377.im;
  c_gradient[1367].re = (ct->f792 - ct->f384.re) + ct->f385.re;
  c_gradient[1367].im = (0.0 - ct->f384.im) + ct->f385.im;
  memset(&c_gradient[1368], 0, 10U * sizeof(creal_T));
  c_gradient[1378].re = (ct->f583 + ct->f590) + ct->f596;
  c_gradient[1378].im = 0.0;
  c_gradient[1379].re = (ct->f779 + ct->f316.re) + ct->f325.re;
  c_gradient[1379].im = ct->f316.im + ct->f325.im;
  c_gradient[1380].re = (ct->f781 + ct->f323.re) + ct->f335.re;
  c_gradient[1380].im = ct->f323.im + ct->f335.im;
  c_gradient[1381].re = (ct->f783 + ct->f333.re) + ct->f343.re;
  c_gradient[1381].im = ct->f333.im + ct->f343.im;
  c_gradient[1382].re = (ct->f785 + ct->f341.re) + ct->f351.re;
  c_gradient[1382].im = ct->f341.im + ct->f351.im;
  c_gradient[1383].re = (ct->f787 + ct->f349.re) + ct->f359.re;
  c_gradient[1383].im = ct->f349.im + ct->f359.im;
  c_gradient[1384].re = (ct->f789 + ct->f357.re) + ct->f367.re;
  c_gradient[1384].im = ct->f357.im + ct->f367.im;
  c_gradient[1385].re = (ct->f790 + ct->f365.re) + ct->f374.re;
  c_gradient[1385].im = ct->f365.im + ct->f374.im;
  c_gradient[1386].re = (ct->f791 + ct->f373.re) + ct->f378.re;
  c_gradient[1386].im = ct->f373.im + ct->f378.im;
  c_gradient[1387].re = (ct->f793 + ct->f382.re) + ct->f386.re;
  c_gradient[1387].im = ct->f382.im + ct->f386.im;
  c_gradient[1388].re = (ct->f577 + ct->f591) - ct->f2 * ct->f105 * ct->f575 *
    2.0;
  c_gradient[1388].im = 0.0;
  c_gradient[1389].re = (ct->f758 + ct->f319.re) - ct->f321.re;
  c_gradient[1389].im = ct->f319.im - ct->f321.im;
  c_gradient[1390].re = (ct->f759 + ct->f329.re) - ct->f331.re;
  c_gradient[1390].im = ct->f329.im - ct->f331.im;
  c_gradient[1391].re = (ct->f760 + ct->f337.re) - ct->f339.re;
  c_gradient[1391].im = ct->f337.im - ct->f339.im;
  c_gradient[1392].re = (ct->f762 + ct->f345.re) - ct->f347.re;
  c_gradient[1392].im = ct->f345.im - ct->f347.im;
  c_gradient[1393].re = (ct->f764 + ct->f353.re) - ct->f355.re;
  c_gradient[1393].im = ct->f353.im - ct->f355.im;
  c_gradient[1394].re = (ct->f766 + ct->f361.re) - ct->f363.re;
  c_gradient[1394].im = ct->f361.im - ct->f363.im;
  c_gradient[1395].re = (ct->f768 + ct->f369.re) - ct->f371.re;
  c_gradient[1395].im = ct->f369.im - ct->f371.im;
  c_gradient[1396].re = (ct->f770 + ct->f376.re) - ct->f377.re;
  c_gradient[1396].im = ct->f376.im - ct->f377.im;
  c_gradient[1397].re = (ct->f774 + ct->f384.re) - ct->f385.re;
  c_gradient[1397].im = ct->f384.im - ct->f385.im;
  memset(&c_gradient[1398], 0, 138U * sizeof(creal_T));
  c_gradient[1536].re = -ct->f565;
  c_gradient[1536].im = 0.0;
  c_gradient[1537].re = -ct->f569;
  c_gradient[1537].im = 0.0;
  c_gradient[1538].re = -ct->f573;
  c_gradient[1538].im = 0.0;
  c_gradient[1539].re = -ct->f592;
  c_gradient[1539].im = 0.0;
  c_gradient[1540].re = -ct->f636;
  c_gradient[1540].im = 0.0;
  c_gradient[1541].re = -ct->f660;
  c_gradient[1541].im = 0.0;
  c_gradient[1542].re = -ct->f668;
  c_gradient[1542].im = 0.0;
  c_gradient[1543].re = -ct->f690;
  c_gradient[1543].im = 0.0;
  c_gradient[1544].re = 0.0;
  c_gradient[1544].im = 0.0;
  c_gradient[1545].re = 0.0;
  c_gradient[1545].im = 0.0;
  c_gradient[1546].re = 0.0;
  c_gradient[1546].im = 0.0;
  c_gradient[1547].re = 0.0;
  c_gradient[1547].im = 0.0;
  c_gradient[1548].re = 0.0;
  c_gradient[1548].im = 0.0;
  c_gradient[1549].re = 0.0;
  c_gradient[1549].im = 0.0;
  c_gradient[1550].re = 0.0;
  c_gradient[1550].im = 0.0;
  c_gradient[1551].re = 0.0;
  c_gradient[1551].im = 0.0;
  c_gradient[1552].re = 0.0;
  c_gradient[1552].im = 0.0;
  c_gradient[1553].re = 0.0;
  c_gradient[1553].im = 0.0;
  c_gradient[1554].re = 0.0;
  c_gradient[1554].im = 0.0;
  c_gradient[1555].re = 0.0;
  c_gradient[1555].im = 0.0;
  c_gradient[1556].re = 0.0;
  c_gradient[1556].im = 0.0;
  c_gradient[1557].re = 0.0;
  c_gradient[1557].im = 0.0;
  c_gradient[1558].re = 0.0;
  c_gradient[1558].im = 0.0;
  c_gradient[1559].re = 0.0;
  c_gradient[1559].im = 0.0;
  c_gradient[1560].re = 0.0;
  c_gradient[1560].im = 0.0;
  c_gradient[1561].re = 0.0;
  c_gradient[1561].im = 0.0;
  c_gradient[1562].re = 0.0;
  c_gradient[1562].im = 0.0;
  c_gradient[1563].re = 0.0;
  c_gradient[1563].im = 0.0;
  c_gradient[1564].re = 0.0;
  c_gradient[1564].im = 0.0;
  c_gradient[1565].re = ct->f494;
  c_gradient[1565].im = 0.0;
  c_gradient[1566].re = ct->f497;
  c_gradient[1566].im = 0.0;
  c_gradient[1567].re = ct->f501;
  c_gradient[1567].im = 0.0;
  c_gradient[1568].re = ct->f504;
  c_gradient[1568].im = 0.0;
  c_gradient[1569].re = ct->f507;
  c_gradient[1569].im = 0.0;
  c_gradient[1570].re = ct->f510;
  c_gradient[1570].im = 0.0;
  c_gradient[1571].re = ct->f512;
  c_gradient[1571].im = 0.0;
  c_gradient[1572].re = ct->f513;
  c_gradient[1572].im = 0.0;
  c_gradient[1573].re = ct->f516;
  c_gradient[1573].im = 0.0;
  c_gradient[1574].re = 0.0;
  c_gradient[1574].im = 0.0;
  c_gradient[1575].re = 0.0;
  c_gradient[1575].im = 0.0;
  c_gradient[1576].re = 0.0;
  c_gradient[1576].im = 0.0;
  c_gradient[1577].re = 0.0;
  c_gradient[1577].im = 0.0;
  c_gradient[1578].re = 0.0;
  c_gradient[1578].im = 0.0;
  c_gradient[1579].re = 0.0;
  c_gradient[1579].im = 0.0;
  c_gradient[1580].re = 0.0;
  c_gradient[1580].im = 0.0;
  c_gradient[1581].re = 0.0;
  c_gradient[1581].im = 0.0;
  c_gradient[1582].re = 0.0;
  c_gradient[1582].im = 0.0;
  c_gradient[1583].re = 0.0;
  c_gradient[1583].im = 0.0;
  c_gradient[1584].re = 0.0;
  c_gradient[1584].im = 0.0;
  c_gradient[1585].re = 0.0;
  c_gradient[1585].im = 0.0;
  c_gradient[1586].re = 0.0;
  c_gradient[1586].im = 0.0;
  c_gradient[1587].re = 0.0;
  c_gradient[1587].im = 0.0;
  c_gradient[1588].re = 0.0;
  c_gradient[1588].im = 0.0;
  c_gradient[1589].re = 0.0;
  c_gradient[1589].im = 0.0;
  c_gradient[1590].re = 0.0;
  c_gradient[1590].im = 0.0;
  c_gradient[1591].re = 0.0;
  c_gradient[1591].im = 0.0;
  c_gradient[1592].re = 0.0;
  c_gradient[1592].im = 0.0;
  c_gradient[1593].re = 0.0;
  c_gradient[1593].im = 0.0;
  c_gradient[1594].re = 0.0;
  c_gradient[1594].im = 0.0;
  c_gradient[1595].re = -ct->f494;
  c_gradient[1595].im = 0.0;
  c_gradient[1596].re = -ct->f497;
  c_gradient[1596].im = 0.0;
  c_gradient[1597].re = -ct->f501;
  c_gradient[1597].im = 0.0;
  c_gradient[1598].re = -ct->f504;
  c_gradient[1598].im = 0.0;
  c_gradient[1599].re = -ct->f507;
  c_gradient[1599].im = 0.0;
  c_gradient[1600].re = -ct->f510;
  c_gradient[1600].im = 0.0;
  c_gradient[1601].re = -ct->f512;
  c_gradient[1601].im = 0.0;
  c_gradient[1602].re = -ct->f513;
  c_gradient[1602].im = 0.0;
  c_gradient[1603].re = -ct->f516;
  c_gradient[1603].im = 0.0;
  c_gradient[1604].re = 0.0;
  c_gradient[1604].im = 0.0;
  c_gradient[1605].re = 0.0;
  c_gradient[1605].im = 0.0;
  c_gradient[1606].re = 0.0;
  c_gradient[1606].im = 0.0;
  c_gradient[1607].re = 0.0;
  c_gradient[1607].im = 0.0;
  c_gradient[1608].re = 0.0;
  c_gradient[1608].im = 0.0;
  c_gradient[1609].re = 0.0;
  c_gradient[1609].im = 0.0;
  c_gradient[1610].re = 0.0;
  c_gradient[1610].im = 0.0;
  c_gradient[1611].re = 0.0;
  c_gradient[1611].im = 0.0;
  c_gradient[1612].re = 0.0;
  c_gradient[1612].im = 0.0;
  c_gradient[1613].re = 0.0;
  c_gradient[1613].im = 0.0;
  c_gradient[1614].re = 0.0;
  c_gradient[1614].im = 0.0;
  c_gradient[1615].re = 0.0;
  c_gradient[1615].im = 0.0;
  c_gradient[1616].re = 0.0;
  c_gradient[1616].im = 0.0;
  c_gradient[1617].re = 0.0;
  c_gradient[1617].im = 0.0;
  c_gradient[1618].re = 0.0;
  c_gradient[1618].im = 0.0;
  c_gradient[1619].re = 0.0;
  c_gradient[1619].im = 0.0;
  c_gradient[1620].re = 0.0;
  c_gradient[1620].im = 0.0;
  c_gradient[1621].re = 0.0;
  c_gradient[1621].im = 0.0;
  c_gradient[1622].re = 0.0;
  c_gradient[1622].im = 0.0;
  c_gradient[1623].re = 0.0;
  c_gradient[1623].im = 0.0;
  c_gradient[1624].re = 0.0;
  c_gradient[1624].im = 0.0;
  c_gradient[1625].re = ct->f531;
  c_gradient[1625].im = 0.0;
  c_gradient[1626].re = ct->f532;
  c_gradient[1626].im = 0.0;
  c_gradient[1627].re = ct->f533;
  c_gradient[1627].im = 0.0;
  c_gradient[1628].re = ct->f534;
  c_gradient[1628].im = 0.0;
  c_gradient[1629].re = ct->f535;
  c_gradient[1629].im = 0.0;
  c_gradient[1630].re = ct->f536;
  c_gradient[1630].im = 0.0;
  c_gradient[1631].re = ct->f537;
  c_gradient[1631].im = 0.0;
  c_gradient[1632].re = ct->f538;
  c_gradient[1632].im = 0.0;
  c_gradient[1633].re = ct->f540;
  c_gradient[1633].im = 0.0;
  c_gradient[1634].re = 0.0;
  c_gradient[1634].im = 0.0;
  c_gradient[1635].re = 0.0;
  c_gradient[1635].im = 0.0;
  c_gradient[1636].re = 0.0;
  c_gradient[1636].im = 0.0;
  c_gradient[1637].re = 0.0;
  c_gradient[1637].im = 0.0;
  c_gradient[1638].re = 0.0;
  c_gradient[1638].im = 0.0;
  c_gradient[1639].re = 0.0;
  c_gradient[1639].im = 0.0;
  c_gradient[1640].re = 0.0;
  c_gradient[1640].im = 0.0;
  c_gradient[1641].re = 0.0;
  c_gradient[1641].im = 0.0;
  c_gradient[1642].re = 0.0;
  c_gradient[1642].im = 0.0;
  c_gradient[1643].re = 0.0;
  c_gradient[1643].im = 0.0;
  c_gradient[1644].re = 0.0;
  c_gradient[1644].im = 0.0;
  c_gradient[1645].re = 0.0;
  c_gradient[1645].im = 0.0;
  c_gradient[1646].re = 0.0;
  c_gradient[1646].im = 0.0;
  c_gradient[1647].re = 0.0;
  c_gradient[1647].im = 0.0;
  c_gradient[1648].re = 0.0;
  c_gradient[1648].im = 0.0;
  c_gradient[1649].re = 0.0;
  c_gradient[1649].im = 0.0;
  c_gradient[1650].re = 0.0;
  c_gradient[1650].im = 0.0;
  c_gradient[1651].re = 0.0;
  c_gradient[1651].im = 0.0;
  c_gradient[1652].re = 0.0;
  c_gradient[1652].im = 0.0;
  c_gradient[1653].re = 0.0;
  c_gradient[1653].im = 0.0;
  c_gradient[1654].re = 0.0;
  c_gradient[1654].im = 0.0;
  c_gradient[1655].re = -ct->f531;
  c_gradient[1655].im = 0.0;
  c_gradient[1656].re = -ct->f532;
  c_gradient[1656].im = 0.0;
  c_gradient[1657].re = -ct->f533;
  c_gradient[1657].im = 0.0;
  c_gradient[1658].re = -ct->f534;
  c_gradient[1658].im = 0.0;
  c_gradient[1659].re = -ct->f535;
  c_gradient[1659].im = 0.0;
  c_gradient[1660].re = -ct->f536;
  c_gradient[1660].im = 0.0;
  c_gradient[1661].re = -ct->f537;
  c_gradient[1661].im = 0.0;
  c_gradient[1662].re = -ct->f538;
  c_gradient[1662].im = 0.0;
  c_gradient[1663].re = -ct->f540;
  c_gradient[1663].im = 0.0;
  c_gradient[1664].re = -ct->f562;
  c_gradient[1664].im = 0.0;
  c_gradient[1665].re = -ct->f566;
  c_gradient[1665].im = 0.0;
  c_gradient[1666].re = -ct->f570;
  c_gradient[1666].im = 0.0;
  c_gradient[1667].re = -ct->f574;
  c_gradient[1667].im = 0.0;
  c_gradient[1668].re = -ct->f603;
  c_gradient[1668].im = 0.0;
  c_gradient[1669].re = -ct->f647;
  c_gradient[1669].im = 0.0;
  c_gradient[1670].re = -ct->f661;
  c_gradient[1670].im = 0.0;
  c_gradient[1671].re = -ct->f679;
  c_gradient[1671].im = 0.0;
  c_gradient[1672].re = 0.0;
  c_gradient[1672].im = 0.0;
  c_gradient[1673].re = 0.0;
  c_gradient[1673].im = 0.0;
  c_gradient[1674].re = 0.0;
  c_gradient[1674].im = 0.0;
  c_gradient[1675].re = 0.0;
  c_gradient[1675].im = 0.0;
  c_gradient[1676].re = 0.0;
  c_gradient[1676].im = 0.0;
  c_gradient[1677].re = 0.0;
  c_gradient[1677].im = 0.0;
  c_gradient[1678].re = 0.0;
  c_gradient[1678].im = 0.0;
  c_gradient[1679].re = 0.0;
  c_gradient[1679].im = 0.0;
  c_gradient[1680].re = 0.0;
  c_gradient[1680].im = 0.0;
  c_gradient[1681].re = 0.0;
  c_gradient[1681].im = 0.0;
  c_gradient[1682].re = 0.0;
  c_gradient[1682].im = 0.0;
  c_gradient[1683].re = 0.0;
  c_gradient[1683].im = 0.0;
  c_gradient[1684].re = 0.0;
  c_gradient[1684].im = 0.0;
  c_gradient[1685].re = 0.0;
  c_gradient[1685].im = 0.0;
  c_gradient[1686].re = 0.0;
  c_gradient[1686].im = 0.0;
  c_gradient[1687].re = 0.0;
  c_gradient[1687].im = 0.0;
  c_gradient[1688].re = 0.0;
  c_gradient[1688].im = 0.0;
  c_gradient[1689].re = 0.0;
  c_gradient[1689].im = 0.0;
  c_gradient[1690].re = 0.0;
  c_gradient[1690].im = 0.0;
  c_gradient[1691].re = 0.0;
  c_gradient[1691].im = 0.0;
  c_gradient[1692].re = 0.0;
  c_gradient[1692].im = 0.0;
  c_gradient[1693].re = ct->f451;
  c_gradient[1693].im = 0.0;
  c_gradient[1694].re = ct->f473;
  c_gradient[1694].im = 0.0;
  c_gradient[1695].re = ct->f495;
  c_gradient[1695].im = 0.0;
  c_gradient[1696].re = ct->f499;
  c_gradient[1696].im = 0.0;
  c_gradient[1697].re = ct->f502;
  c_gradient[1697].im = 0.0;
  c_gradient[1698].re = ct->f505;
  c_gradient[1698].im = 0.0;
  c_gradient[1699].re = ct->f508;
  c_gradient[1699].im = 0.0;
  c_gradient[1700].re = ct->f511;
  c_gradient[1700].im = 0.0;
  c_gradient[1701].re = ct->f515;
  c_gradient[1701].im = 0.0;
  c_gradient[1702].re = 0.0;
  c_gradient[1702].im = 0.0;
  c_gradient[1703].re = 0.0;
  c_gradient[1703].im = 0.0;
  c_gradient[1704].re = 0.0;
  c_gradient[1704].im = 0.0;
  c_gradient[1705].re = 0.0;
  c_gradient[1705].im = 0.0;
  c_gradient[1706].re = 0.0;
  c_gradient[1706].im = 0.0;
  c_gradient[1707].re = 0.0;
  c_gradient[1707].im = 0.0;
  c_gradient[1708].re = 0.0;
  c_gradient[1708].im = 0.0;
  c_gradient[1709].re = 0.0;
  c_gradient[1709].im = 0.0;
  c_gradient[1710].re = 0.0;
  c_gradient[1710].im = 0.0;
  c_gradient[1711].re = 0.0;
  c_gradient[1711].im = 0.0;
  c_gradient[1712].re = 0.0;
  c_gradient[1712].im = 0.0;
  c_gradient[1713].re = 0.0;
  c_gradient[1713].im = 0.0;
  c_gradient[1714].re = 0.0;
  c_gradient[1714].im = 0.0;
  c_gradient[1715].re = 0.0;
  c_gradient[1715].im = 0.0;
  c_gradient[1716].re = 0.0;
  c_gradient[1716].im = 0.0;
  c_gradient[1717].re = 0.0;
  c_gradient[1717].im = 0.0;
  c_gradient[1718].re = 0.0;
  c_gradient[1718].im = 0.0;
  c_gradient[1719].re = 0.0;
  c_gradient[1719].im = 0.0;
  c_gradient[1720].re = 0.0;
  c_gradient[1720].im = 0.0;
  c_gradient[1721].re = 0.0;
  c_gradient[1721].im = 0.0;
  c_gradient[1722].re = 0.0;
  c_gradient[1722].im = 0.0;
  c_gradient[1723].re = -ct->f451;
  c_gradient[1723].im = 0.0;
  c_gradient[1724].re = -ct->f473;
  c_gradient[1724].im = 0.0;
  c_gradient[1725].re = -ct->f495;
  c_gradient[1725].im = 0.0;
  c_gradient[1726].re = -ct->f499;
  c_gradient[1726].im = 0.0;
  c_gradient[1727].re = -ct->f502;
  c_gradient[1727].im = 0.0;
  c_gradient[1728].re = -ct->f505;
  c_gradient[1728].im = 0.0;
  c_gradient[1729].re = -ct->f508;
  c_gradient[1729].im = 0.0;
  c_gradient[1730].re = -ct->f511;
  c_gradient[1730].im = 0.0;
  c_gradient[1731].re = -ct->f515;
  c_gradient[1731].im = 0.0;
  c_gradient[1732].re = 0.0;
  c_gradient[1732].im = 0.0;
  c_gradient[1733].re = 0.0;
  c_gradient[1733].im = 0.0;
  c_gradient[1734].re = 0.0;
  c_gradient[1734].im = 0.0;
  c_gradient[1735].re = 0.0;
  c_gradient[1735].im = 0.0;
  c_gradient[1736].re = 0.0;
  c_gradient[1736].im = 0.0;
  c_gradient[1737].re = 0.0;
  c_gradient[1737].im = 0.0;
  c_gradient[1738].re = 0.0;
  c_gradient[1738].im = 0.0;
  c_gradient[1739].re = 0.0;
  c_gradient[1739].im = 0.0;
  c_gradient[1740].re = 0.0;
  c_gradient[1740].im = 0.0;
  c_gradient[1741].re = 0.0;
  c_gradient[1741].im = 0.0;
  c_gradient[1742].re = 0.0;
  c_gradient[1742].im = 0.0;
  c_gradient[1743].re = 0.0;
  c_gradient[1743].im = 0.0;
  c_gradient[1744].re = 0.0;
  c_gradient[1744].im = 0.0;
  c_gradient[1745].re = 0.0;
  c_gradient[1745].im = 0.0;
  c_gradient[1746].re = 0.0;
  c_gradient[1746].im = 0.0;
  c_gradient[1747].re = 0.0;
  c_gradient[1747].im = 0.0;
  c_gradient[1748].re = 0.0;
  c_gradient[1748].im = 0.0;
  c_gradient[1749].re = 0.0;
  c_gradient[1749].im = 0.0;
  c_gradient[1750].re = 0.0;
  c_gradient[1750].im = 0.0;
  c_gradient[1751].re = 0.0;
  c_gradient[1751].im = 0.0;
  c_gradient[1752].re = 0.0;
  c_gradient[1752].im = 0.0;
  c_gradient[1753].re = ct->f522;
  c_gradient[1753].im = 0.0;
  c_gradient[1754].re = ct->f523;
  c_gradient[1754].im = 0.0;
  c_gradient[1755].re = ct->f524;
  c_gradient[1755].im = 0.0;
  c_gradient[1756].re = ct->f525;
  c_gradient[1756].im = 0.0;
  c_gradient[1757].re = ct->f526;
  c_gradient[1757].im = 0.0;
  c_gradient[1758].re = ct->f527;
  c_gradient[1758].im = 0.0;
  c_gradient[1759].re = ct->f528;
  c_gradient[1759].im = 0.0;
  c_gradient[1760].re = ct->f529;
  c_gradient[1760].im = 0.0;
  c_gradient[1761].re = ct->f539;
  c_gradient[1761].im = 0.0;
  c_gradient[1762].re = 0.0;
  c_gradient[1762].im = 0.0;
  c_gradient[1763].re = 0.0;
  c_gradient[1763].im = 0.0;
  c_gradient[1764].re = 0.0;
  c_gradient[1764].im = 0.0;
  c_gradient[1765].re = 0.0;
  c_gradient[1765].im = 0.0;
  c_gradient[1766].re = 0.0;
  c_gradient[1766].im = 0.0;
  c_gradient[1767].re = 0.0;
  c_gradient[1767].im = 0.0;
  c_gradient[1768].re = 0.0;
  c_gradient[1768].im = 0.0;
  c_gradient[1769].re = 0.0;
  c_gradient[1769].im = 0.0;
  c_gradient[1770].re = 0.0;
  c_gradient[1770].im = 0.0;
  c_gradient[1771].re = 0.0;
  c_gradient[1771].im = 0.0;
  c_gradient[1772].re = 0.0;
  c_gradient[1772].im = 0.0;
  c_gradient[1773].re = 0.0;
  c_gradient[1773].im = 0.0;
  c_gradient[1774].re = 0.0;
  c_gradient[1774].im = 0.0;
  c_gradient[1775].re = 0.0;
  c_gradient[1775].im = 0.0;
  c_gradient[1776].re = 0.0;
  c_gradient[1776].im = 0.0;
  c_gradient[1777].re = 0.0;
  c_gradient[1777].im = 0.0;
  c_gradient[1778].re = 0.0;
  c_gradient[1778].im = 0.0;
  c_gradient[1779].re = 0.0;
  c_gradient[1779].im = 0.0;
  c_gradient[1780].re = 0.0;
  c_gradient[1780].im = 0.0;
  c_gradient[1781].re = 0.0;
  c_gradient[1781].im = 0.0;
  c_gradient[1782].re = 0.0;
  c_gradient[1782].im = 0.0;
  c_gradient[1783].re = -ct->f522;
  c_gradient[1783].im = 0.0;
  c_gradient[1784].re = -ct->f523;
  c_gradient[1784].im = 0.0;
  c_gradient[1785].re = -ct->f524;
  c_gradient[1785].im = 0.0;
  c_gradient[1786].re = -ct->f525;
  c_gradient[1786].im = 0.0;
  c_gradient[1787].re = -ct->f526;
  c_gradient[1787].im = 0.0;
  c_gradient[1788].re = -ct->f527;
  c_gradient[1788].im = 0.0;
  c_gradient[1789].re = -ct->f528;
  c_gradient[1789].im = 0.0;
  c_gradient[1790].re = -ct->f529;
  c_gradient[1790].im = 0.0;
  c_gradient[1791].re = -ct->f539;
  c_gradient[1791].im = 0.0;
  c_gradient[1792].re = -ct->f558;
  c_gradient[1792].im = 0.0;
  c_gradient[1793].re = -ct->f563;
  c_gradient[1793].im = 0.0;
  c_gradient[1794].re = -ct->f567;
  c_gradient[1794].im = 0.0;
  c_gradient[1795].re = -ct->f571;
  c_gradient[1795].im = 0.0;
  c_gradient[1796].re = -ct->f576;
  c_gradient[1796].im = 0.0;
  c_gradient[1797].re = -ct->f614;
  c_gradient[1797].im = 0.0;
  c_gradient[1798].re = -ct->f658;
  c_gradient[1798].im = 0.0;
  c_gradient[1799].re = -ct->f662;
  c_gradient[1799].im = 0.0;
  c_gradient[1800].re = 0.0;
  c_gradient[1800].im = 0.0;
  c_gradient[1801].re = 0.0;
  c_gradient[1801].im = 0.0;
  c_gradient[1802].re = 0.0;
  c_gradient[1802].im = 0.0;
  c_gradient[1803].re = 0.0;
  c_gradient[1803].im = 0.0;
  c_gradient[1804].re = 0.0;
  c_gradient[1804].im = 0.0;
  c_gradient[1805].re = 0.0;
  c_gradient[1805].im = 0.0;
  c_gradient[1806].re = 0.0;
  c_gradient[1806].im = 0.0;
  c_gradient[1807].re = 0.0;
  c_gradient[1807].im = 0.0;
  c_gradient[1808].re = 0.0;
  c_gradient[1808].im = 0.0;
  c_gradient[1809].re = 0.0;
  c_gradient[1809].im = 0.0;
  c_gradient[1810].re = 0.0;
  c_gradient[1810].im = 0.0;
  c_gradient[1811].re = 0.0;
  c_gradient[1811].im = 0.0;
  c_gradient[1812].re = 0.0;
  c_gradient[1812].im = 0.0;
  c_gradient[1813].re = 0.0;
  c_gradient[1813].im = 0.0;
  c_gradient[1814].re = 0.0;
  c_gradient[1814].im = 0.0;
  c_gradient[1815].re = 0.0;
  c_gradient[1815].im = 0.0;
  c_gradient[1816].re = 0.0;
  c_gradient[1816].im = 0.0;
  c_gradient[1817].re = 0.0;
  c_gradient[1817].im = 0.0;
  c_gradient[1818].re = 0.0;
  c_gradient[1818].im = 0.0;
  c_gradient[1819].re = 0.0;
  c_gradient[1819].im = 0.0;
  c_gradient[1820].re = 0.0;
  c_gradient[1820].im = 0.0;
  c_gradient[1821].re = ct->f429;
  c_gradient[1821].im = 0.0;
  c_gradient[1822].re = ct->f440;
  c_gradient[1822].im = 0.0;
  c_gradient[1823].re = ct->f462;
  c_gradient[1823].im = 0.0;
  c_gradient[1824].re = ct->f484;
  c_gradient[1824].im = 0.0;
  c_gradient[1825].re = ct->f496;
  c_gradient[1825].im = 0.0;
  c_gradient[1826].re = ct->f500;
  c_gradient[1826].im = 0.0;
  c_gradient[1827].re = ct->f503;
  c_gradient[1827].im = 0.0;
  c_gradient[1828].re = ct->f506;
  c_gradient[1828].im = 0.0;
  c_gradient[1829].re = ct->f514;
  c_gradient[1829].im = 0.0;
  c_gradient[1830].re = 0.0;
  c_gradient[1830].im = 0.0;
  c_gradient[1831].re = 0.0;
  c_gradient[1831].im = 0.0;
  c_gradient[1832].re = 0.0;
  c_gradient[1832].im = 0.0;
  c_gradient[1833].re = 0.0;
  c_gradient[1833].im = 0.0;
  c_gradient[1834].re = 0.0;
  c_gradient[1834].im = 0.0;
  c_gradient[1835].re = 0.0;
  c_gradient[1835].im = 0.0;
  c_gradient[1836].re = 0.0;
  c_gradient[1836].im = 0.0;
  c_gradient[1837].re = 0.0;
  c_gradient[1837].im = 0.0;
  c_gradient[1838].re = 0.0;
  c_gradient[1838].im = 0.0;
  c_gradient[1839].re = 0.0;
  c_gradient[1839].im = 0.0;
  c_gradient[1840].re = 0.0;
  c_gradient[1840].im = 0.0;
  c_gradient[1841].re = 0.0;
  c_gradient[1841].im = 0.0;
  c_gradient[1842].re = 0.0;
  c_gradient[1842].im = 0.0;
  c_gradient[1843].re = 0.0;
  c_gradient[1843].im = 0.0;
  c_gradient[1844].re = 0.0;
  c_gradient[1844].im = 0.0;
  c_gradient[1845].re = 0.0;
  c_gradient[1845].im = 0.0;
  c_gradient[1846].re = 0.0;
  c_gradient[1846].im = 0.0;
  c_gradient[1847].re = 0.0;
  c_gradient[1847].im = 0.0;
  c_gradient[1848].re = 0.0;
  c_gradient[1848].im = 0.0;
  c_gradient[1849].re = 0.0;
  c_gradient[1849].im = 0.0;
  c_gradient[1850].re = 0.0;
  c_gradient[1850].im = 0.0;
  c_gradient[1851].re = -ct->f429;
  c_gradient[1851].im = 0.0;
  c_gradient[1852].re = -ct->f440;
  c_gradient[1852].im = 0.0;
  c_gradient[1853].re = -ct->f462;
  c_gradient[1853].im = 0.0;
  c_gradient[1854].re = -ct->f484;
  c_gradient[1854].im = 0.0;
  c_gradient[1855].re = -ct->f496;
  c_gradient[1855].im = 0.0;
  c_gradient[1856].re = -ct->f500;
  c_gradient[1856].im = 0.0;
  c_gradient[1857].re = -ct->f503;
  c_gradient[1857].im = 0.0;
  c_gradient[1858].re = -ct->f506;
  c_gradient[1858].im = 0.0;
  c_gradient[1859].re = -ct->f514;
  c_gradient[1859].im = 0.0;
  c_gradient[1860].re = 0.0;
  c_gradient[1860].im = 0.0;
  c_gradient[1861].re = 0.0;
  c_gradient[1861].im = 0.0;
  c_gradient[1862].re = 0.0;
  c_gradient[1862].im = 0.0;
  c_gradient[1863].re = 0.0;
  c_gradient[1863].im = 0.0;
  c_gradient[1864].re = 0.0;
  c_gradient[1864].im = 0.0;
  c_gradient[1865].re = 0.0;
  c_gradient[1865].im = 0.0;
  c_gradient[1866].re = 0.0;
  c_gradient[1866].im = 0.0;
  c_gradient[1867].re = 0.0;
  c_gradient[1867].im = 0.0;
  c_gradient[1868].re = 0.0;
  c_gradient[1868].im = 0.0;
  c_gradient[1869].re = 0.0;
  c_gradient[1869].im = 0.0;
  c_gradient[1870].re = 0.0;
  c_gradient[1870].im = 0.0;
  c_gradient[1871].re = 0.0;
  c_gradient[1871].im = 0.0;
  c_gradient[1872].re = 0.0;
  c_gradient[1872].im = 0.0;
  c_gradient[1873].re = 0.0;
  c_gradient[1873].im = 0.0;
  c_gradient[1874].re = 0.0;
  c_gradient[1874].im = 0.0;
  c_gradient[1875].re = 0.0;
  c_gradient[1875].im = 0.0;
  c_gradient[1876].re = 0.0;
  c_gradient[1876].im = 0.0;
  c_gradient[1877].re = 0.0;
  c_gradient[1877].im = 0.0;
  c_gradient[1878].re = 0.0;
  c_gradient[1878].im = 0.0;
  c_gradient[1879].re = 0.0;
  c_gradient[1879].im = 0.0;
  c_gradient[1880].re = 0.0;
  c_gradient[1880].im = 0.0;
  c_gradient[1881].re = ct->f509;
  c_gradient[1881].im = 0.0;
  c_gradient[1882].re = ct->f530;
  c_gradient[1882].im = 0.0;
  c_gradient[1883].re = ct->f543;
  c_gradient[1883].im = 0.0;
  c_gradient[1884].re = ct->f545;
  c_gradient[1884].im = 0.0;
  c_gradient[1885].re = ct->f547;
  c_gradient[1885].im = 0.0;
  c_gradient[1886].re = ct->f548;
  c_gradient[1886].im = 0.0;
  c_gradient[1887].re = ct->f549;
  c_gradient[1887].im = 0.0;
  c_gradient[1888].re = ct->f550;
  c_gradient[1888].im = 0.0;
  c_gradient[1889].re = ct->f553;
  c_gradient[1889].im = 0.0;
  c_gradient[1890].re = 0.0;
  c_gradient[1890].im = 0.0;
  c_gradient[1891].re = 0.0;
  c_gradient[1891].im = 0.0;
  c_gradient[1892].re = 0.0;
  c_gradient[1892].im = 0.0;
  c_gradient[1893].re = 0.0;
  c_gradient[1893].im = 0.0;
  c_gradient[1894].re = 0.0;
  c_gradient[1894].im = 0.0;
  c_gradient[1895].re = 0.0;
  c_gradient[1895].im = 0.0;
  c_gradient[1896].re = 0.0;
  c_gradient[1896].im = 0.0;
  c_gradient[1897].re = 0.0;
  c_gradient[1897].im = 0.0;
  c_gradient[1898].re = 0.0;
  c_gradient[1898].im = 0.0;
  c_gradient[1899].re = 0.0;
  c_gradient[1899].im = 0.0;
  c_gradient[1900].re = 0.0;
  c_gradient[1900].im = 0.0;
  c_gradient[1901].re = 0.0;
  c_gradient[1901].im = 0.0;
  c_gradient[1902].re = 0.0;
  c_gradient[1902].im = 0.0;
  c_gradient[1903].re = 0.0;
  c_gradient[1903].im = 0.0;
  c_gradient[1904].re = 0.0;
  c_gradient[1904].im = 0.0;
  c_gradient[1905].re = 0.0;
  c_gradient[1905].im = 0.0;
  c_gradient[1906].re = 0.0;
  c_gradient[1906].im = 0.0;
  c_gradient[1907].re = 0.0;
  c_gradient[1907].im = 0.0;
  c_gradient[1908].re = 0.0;
  c_gradient[1908].im = 0.0;
  c_gradient[1909].re = 0.0;
  c_gradient[1909].im = 0.0;
  c_gradient[1910].re = 0.0;
  c_gradient[1910].im = 0.0;
  c_gradient[1911].re = -ct->f509;
  c_gradient[1911].im = 0.0;
  c_gradient[1912].re = -ct->f530;
  c_gradient[1912].im = 0.0;
  c_gradient[1913].re = -ct->f543;
  c_gradient[1913].im = 0.0;
  c_gradient[1914].re = -ct->f545;
  c_gradient[1914].im = 0.0;
  c_gradient[1915].re = -ct->f547;
  c_gradient[1915].im = 0.0;
  c_gradient[1916].re = -ct->f548;
  c_gradient[1916].im = 0.0;
  c_gradient[1917].re = -ct->f549;
  c_gradient[1917].im = 0.0;
  c_gradient[1918].re = -ct->f550;
  c_gradient[1918].im = 0.0;
  c_gradient[1919].re = -ct->f553;
  c_gradient[1919].im = 0.0;
  c_gradient[1920].re = -ct->f556;
  c_gradient[1920].im = 0.0;
  c_gradient[1921].re = -ct->f561;
  c_gradient[1921].im = 0.0;
  c_gradient[1922].re = -ct->f564;
  c_gradient[1922].im = 0.0;
  c_gradient[1923].re = -ct->f568;
  c_gradient[1923].im = 0.0;
  c_gradient[1924].re = -ct->f572;
  c_gradient[1924].im = 0.0;
  c_gradient[1925].re = -ct->f586;
  c_gradient[1925].im = 0.0;
  c_gradient[1926].re = -ct->f625;
  c_gradient[1926].im = 0.0;
  c_gradient[1927].re = -ct->f659;
  c_gradient[1927].im = 0.0;
  c_gradient[1928].re = 0.0;
  c_gradient[1928].im = 0.0;
  c_gradient[1929].re = 0.0;
  c_gradient[1929].im = 0.0;
  c_gradient[1930].re = 0.0;
  c_gradient[1930].im = 0.0;
  c_gradient[1931].re = 0.0;
  c_gradient[1931].im = 0.0;
  c_gradient[1932].re = 0.0;
  c_gradient[1932].im = 0.0;
  c_gradient[1933].re = 0.0;
  c_gradient[1933].im = 0.0;
  c_gradient[1934].re = 0.0;
  c_gradient[1934].im = 0.0;
  c_gradient[1935].re = 0.0;
  c_gradient[1935].im = 0.0;
  c_gradient[1936].re = 0.0;
  c_gradient[1936].im = 0.0;
  c_gradient[1937].re = 0.0;
  c_gradient[1937].im = 0.0;
  c_gradient[1938].re = 0.0;
  c_gradient[1938].im = 0.0;
  c_gradient[1939].re = 0.0;
  c_gradient[1939].im = 0.0;
  c_gradient[1940].re = 0.0;
  c_gradient[1940].im = 0.0;
  c_gradient[1941].re = 0.0;
  c_gradient[1941].im = 0.0;
  c_gradient[1942].re = 0.0;
  c_gradient[1942].im = 0.0;
  c_gradient[1943].re = 0.0;
  c_gradient[1943].im = 0.0;
  c_gradient[1944].re = 0.0;
  c_gradient[1944].im = 0.0;
  c_gradient[1945].re = 0.0;
  c_gradient[1945].im = 0.0;
  c_gradient[1946].re = 0.0;
  c_gradient[1946].im = 0.0;
  c_gradient[1947].re = 0.0;
  c_gradient[1947].im = 0.0;
  c_gradient[1948].re = 0.0;
  c_gradient[1948].im = 0.0;
  c_gradient[1949].re = ct->f261;
  c_gradient[1949].im = 0.0;
  c_gradient[1950].re = ct->f326;
  c_gradient[1950].im = 0.0;
  c_gradient[1951].re = ct->f428;
  c_gradient[1951].im = 0.0;
  c_gradient[1952].re = ct->f498;
  c_gradient[1952].im = 0.0;
  c_gradient[1953].re = ct->f520;
  c_gradient[1953].im = 0.0;
  c_gradient[1954].re = ct->f542;
  c_gradient[1954].im = 0.0;
  c_gradient[1955].re = ct->f544;
  c_gradient[1955].im = 0.0;
  c_gradient[1956].re = ct->f546;
  c_gradient[1956].im = 0.0;
  c_gradient[1957].re = ct->f551;
  c_gradient[1957].im = 0.0;
  c_gradient[1958].re = 0.0;
  c_gradient[1958].im = 0.0;
  c_gradient[1959].re = 0.0;
  c_gradient[1959].im = 0.0;
  c_gradient[1960].re = 0.0;
  c_gradient[1960].im = 0.0;
  c_gradient[1961].re = 0.0;
  c_gradient[1961].im = 0.0;
  c_gradient[1962].re = 0.0;
  c_gradient[1962].im = 0.0;
  c_gradient[1963].re = 0.0;
  c_gradient[1963].im = 0.0;
  c_gradient[1964].re = 0.0;
  c_gradient[1964].im = 0.0;
  c_gradient[1965].re = 0.0;
  c_gradient[1965].im = 0.0;
  c_gradient[1966].re = 0.0;
  c_gradient[1966].im = 0.0;
  c_gradient[1967].re = 0.0;
  c_gradient[1967].im = 0.0;
  c_gradient[1968].re = 0.0;
  c_gradient[1968].im = 0.0;
  c_gradient[1969].re = 0.0;
  c_gradient[1969].im = 0.0;
  c_gradient[1970].re = 0.0;
  c_gradient[1970].im = 0.0;
  c_gradient[1971].re = 0.0;
  c_gradient[1971].im = 0.0;
  c_gradient[1972].re = 0.0;
  c_gradient[1972].im = 0.0;
  c_gradient[1973].re = 0.0;
  c_gradient[1973].im = 0.0;
  c_gradient[1974].re = 0.0;
  c_gradient[1974].im = 0.0;
  c_gradient[1975].re = 0.0;
  c_gradient[1975].im = 0.0;
  c_gradient[1976].re = 0.0;
  c_gradient[1976].im = 0.0;
  c_gradient[1977].re = 0.0;
  c_gradient[1977].im = 0.0;
  c_gradient[1978].re = 0.0;
  c_gradient[1978].im = 0.0;
  c_gradient[1979].re = -ct->f261;
  c_gradient[1979].im = 0.0;
  c_gradient[1980].re = -ct->f326;
  c_gradient[1980].im = 0.0;
  c_gradient[1981].re = -ct->f428;
  c_gradient[1981].im = 0.0;
  c_gradient[1982].re = -ct->f498;
  c_gradient[1982].im = 0.0;
  c_gradient[1983].re = -ct->f520;
  c_gradient[1983].im = 0.0;
  c_gradient[1984].re = -ct->f542;
  c_gradient[1984].im = 0.0;
  c_gradient[1985].re = -ct->f544;
  c_gradient[1985].im = 0.0;
  c_gradient[1986].re = -ct->f546;
  c_gradient[1986].im = 0.0;
  c_gradient[1987].re = -ct->f551;
  c_gradient[1987].im = 0.0;
  c_gradient[1988].re = 0.0;
  c_gradient[1988].im = 0.0;
  c_gradient[1989].re = 0.0;
  c_gradient[1989].im = 0.0;
  c_gradient[1990].re = 0.0;
  c_gradient[1990].im = 0.0;
  c_gradient[1991].re = 0.0;
  c_gradient[1991].im = 0.0;
  c_gradient[1992].re = 0.0;
  c_gradient[1992].im = 0.0;
  c_gradient[1993].re = 0.0;
  c_gradient[1993].im = 0.0;
  c_gradient[1994].re = 0.0;
  c_gradient[1994].im = 0.0;
  c_gradient[1995].re = 0.0;
  c_gradient[1995].im = 0.0;
  c_gradient[1996].re = 0.0;
  c_gradient[1996].im = 0.0;
  c_gradient[1997].re = 0.0;
  c_gradient[1997].im = 0.0;
  c_gradient[1998].re = 0.0;
  c_gradient[1998].im = 0.0;
  c_gradient[1999].re = 0.0;
  c_gradient[1999].im = 0.0;
  c_gradient[2000].re = 0.0;
  c_gradient[2000].im = 0.0;
  c_gradient[2001].re = 0.0;
  c_gradient[2001].im = 0.0;
  c_gradient[2002].re = 0.0;
  c_gradient[2002].im = 0.0;
  c_gradient[2003].re = 0.0;
  c_gradient[2003].im = 0.0;
  c_gradient[2004].re = 0.0;
  c_gradient[2004].im = 0.0;
  c_gradient[2005].re = 0.0;
  c_gradient[2005].im = 0.0;
  c_gradient[2006].re = 0.0;
  c_gradient[2006].im = 0.0;
  c_gradient[2007].re = 0.0;
  c_gradient[2007].im = 0.0;
  c_gradient[2008].re = 2.0;
  c_gradient[2008].im = 0.0;
  c_gradient[2009].re = 2.0;
  c_gradient[2009].im = 0.0;
  c_gradient[2010].re = 2.0;
  c_gradient[2010].im = 0.0;
  c_gradient[2011].re = 2.0;
  c_gradient[2011].im = 0.0;
  c_gradient[2012].re = 2.0;
  c_gradient[2012].im = 0.0;
  c_gradient[2013].re = 2.0;
  c_gradient[2013].im = 0.0;
  c_gradient[2014].re = 2.0;
  c_gradient[2014].im = 0.0;
  c_gradient[2015].re = 2.0;
  c_gradient[2015].im = 0.0;
  c_gradient[2016].re = 2.0;
  c_gradient[2016].im = 0.0;
  c_gradient[2017].re = 2.0;
  c_gradient[2017].im = 0.0;
  c_gradient[2018].re = 0.0;
  c_gradient[2018].im = 0.0;
  c_gradient[2019].re = 0.0;
  c_gradient[2019].im = 0.0;
  c_gradient[2020].re = 0.0;
  c_gradient[2020].im = 0.0;
  c_gradient[2021].re = 0.0;
  c_gradient[2021].im = 0.0;
  c_gradient[2022].re = 0.0;
  c_gradient[2022].im = 0.0;
  c_gradient[2023].re = 0.0;
  c_gradient[2023].im = 0.0;
  c_gradient[2024].re = 0.0;
  c_gradient[2024].im = 0.0;
  c_gradient[2025].re = 0.0;
  c_gradient[2025].im = 0.0;
  c_gradient[2026].re = 0.0;
  c_gradient[2026].im = 0.0;
  c_gradient[2027].re = 0.0;
  c_gradient[2027].im = 0.0;
  c_gradient[2028].re = 0.0;
  c_gradient[2028].im = 0.0;
  c_gradient[2029].re = 0.0;
  c_gradient[2029].im = 0.0;
  c_gradient[2030].re = 0.0;
  c_gradient[2030].im = 0.0;
  c_gradient[2031].re = 0.0;
  c_gradient[2031].im = 0.0;
  c_gradient[2032].re = 0.0;
  c_gradient[2032].im = 0.0;
  c_gradient[2033].re = 0.0;
  c_gradient[2033].im = 0.0;
  c_gradient[2034].re = 0.0;
  c_gradient[2034].im = 0.0;
  c_gradient[2035].re = 0.0;
  c_gradient[2035].im = 0.0;
  c_gradient[2036].re = 0.0;
  c_gradient[2036].im = 0.0;
  c_gradient[2037].re = 0.0;
  c_gradient[2037].im = 0.0;
  c_gradient[2038].re = -2.0;
  c_gradient[2038].im = 0.0;
  c_gradient[2039].re = -2.0;
  c_gradient[2039].im = 0.0;
  c_gradient[2040].re = -2.0;
  c_gradient[2040].im = 0.0;
  c_gradient[2041].re = -2.0;
  c_gradient[2041].im = 0.0;
  c_gradient[2042].re = -2.0;
  c_gradient[2042].im = 0.0;
  c_gradient[2043].re = -2.0;
  c_gradient[2043].im = 0.0;
  c_gradient[2044].re = -2.0;
  c_gradient[2044].im = 0.0;
  c_gradient[2045].re = -2.0;
  c_gradient[2045].im = 0.0;
  c_gradient[2046].re = -2.0;
  c_gradient[2046].im = 0.0;
  c_gradient[2047].re = -2.0;
  c_gradient[2047].im = 0.0;
  c_gradient[2048].re = -ct->f948;
  c_gradient[2048].im = 0.0;
  c_gradient[2049].re = -ct->f949;
  c_gradient[2049].im = 0.0;
  c_gradient[2050].re = -ct->f950;
  c_gradient[2050].im = 0.0;
  c_gradient[2051].re = -ct->f951;
  c_gradient[2051].im = 0.0;
  c_gradient[2052].re = -ct->f952;
  c_gradient[2052].im = 0.0;
  c_gradient[2053].re = -ct->f953;
  c_gradient[2053].im = 0.0;
  c_gradient[2054].re = -ct->f954;
  c_gradient[2054].im = 0.0;
  c_gradient[2055].re = -ct->f955;
  c_gradient[2055].im = 0.0;
  c_gradient[2056].re = 0.0;
  c_gradient[2056].im = 0.0;
  c_gradient[2057].re = 0.0;
  c_gradient[2057].im = 0.0;
  c_gradient[2058].re = 0.0;
  c_gradient[2058].im = 0.0;
  c_gradient[2059].re = 0.0;
  c_gradient[2059].im = 0.0;
  c_gradient[2060].re = 0.0;
  c_gradient[2060].im = 0.0;
  c_gradient[2061].re = 0.0;
  c_gradient[2061].im = 0.0;
  c_gradient[2062].re = 0.0;
  c_gradient[2062].im = 0.0;
  c_gradient[2063].re = 0.0;
  c_gradient[2063].im = 0.0;
  c_gradient[2064].re = 0.0;
  c_gradient[2064].im = 0.0;
  c_gradient[2065].re = 0.0;
  c_gradient[2065].im = 0.0;
  c_gradient[2066].re = 0.0;
  c_gradient[2066].im = 0.0;
  c_gradient[2067].re = 0.0;
  c_gradient[2067].im = 0.0;
  c_gradient[2068].re = 0.0;
  c_gradient[2068].im = 0.0;
  c_gradient[2069].re = 0.0;
  c_gradient[2069].im = 0.0;
  c_gradient[2070].re = 0.0;
  c_gradient[2070].im = 0.0;
  c_gradient[2071].re = 0.0;
  c_gradient[2071].im = 0.0;
  c_gradient[2072].re = 0.0;
  c_gradient[2072].im = 0.0;
  c_gradient[2073].re = 0.0;
  c_gradient[2073].im = 0.0;
  c_gradient[2074].re = 0.0;
  c_gradient[2074].im = 0.0;
  c_gradient[2075].re = 0.0;
  c_gradient[2075].im = 0.0;
  c_gradient[2076].re = 1.0;
  c_gradient[2076].im = 0.0;
  c_gradient[2077].re = 1.0;
  c_gradient[2077].im = 0.0;
  c_gradient[2078].re = 1.0;
  c_gradient[2078].im = 0.0;
  c_gradient[2079].re = 1.0;
  c_gradient[2079].im = 0.0;
  c_gradient[2080].re = 1.0;
  c_gradient[2080].im = 0.0;
  c_gradient[2081].re = 1.0;
  c_gradient[2081].im = 0.0;
  c_gradient[2082].re = 1.0;
  c_gradient[2082].im = 0.0;
  c_gradient[2083].re = 1.0;
  c_gradient[2083].im = 0.0;
  c_gradient[2084].re = 1.0;
  c_gradient[2084].im = 0.0;
  c_gradient[2085].re = 1.0;
  c_gradient[2085].im = 0.0;
  c_gradient[2086].re = 0.0;
  c_gradient[2086].im = 0.0;
  c_gradient[2087].re = 0.0;
  c_gradient[2087].im = 0.0;
  c_gradient[2088].re = 0.0;
  c_gradient[2088].im = 0.0;
  c_gradient[2089].re = 0.0;
  c_gradient[2089].im = 0.0;
  c_gradient[2090].re = 0.0;
  c_gradient[2090].im = 0.0;
  c_gradient[2091].re = 0.0;
  c_gradient[2091].im = 0.0;
  c_gradient[2092].re = 0.0;
  c_gradient[2092].im = 0.0;
  c_gradient[2093].re = 0.0;
  c_gradient[2093].im = 0.0;
  c_gradient[2094].re = 0.0;
  c_gradient[2094].im = 0.0;
  c_gradient[2095].re = 0.0;
  c_gradient[2095].im = 0.0;
  c_gradient[2096].re = 0.0;
  c_gradient[2096].im = 0.0;
  c_gradient[2097].re = 0.0;
  c_gradient[2097].im = 0.0;
  c_gradient[2098].re = 0.0;
  c_gradient[2098].im = 0.0;
  c_gradient[2099].re = 0.0;
  c_gradient[2099].im = 0.0;
  c_gradient[2100].re = 0.0;
  c_gradient[2100].im = 0.0;
  c_gradient[2101].re = 0.0;
  c_gradient[2101].im = 0.0;
  c_gradient[2102].re = 0.0;
  c_gradient[2102].im = 0.0;
  c_gradient[2103].re = 0.0;
  c_gradient[2103].im = 0.0;
  c_gradient[2104].re = 0.0;
  c_gradient[2104].im = 0.0;
  c_gradient[2105].re = 0.0;
  c_gradient[2105].im = 0.0;
  c_gradient[2106].re = -1.0;
  c_gradient[2106].im = 0.0;
  c_gradient[2107].re = -1.0;
  c_gradient[2107].im = 0.0;
  c_gradient[2108].re = -1.0;
  c_gradient[2108].im = 0.0;
  c_gradient[2109].re = -1.0;
  c_gradient[2109].im = 0.0;
  c_gradient[2110].re = -1.0;
  c_gradient[2110].im = 0.0;
  c_gradient[2111].re = -1.0;
  c_gradient[2111].im = 0.0;
  c_gradient[2112].re = -1.0;
  c_gradient[2112].im = 0.0;
  c_gradient[2113].re = -1.0;
  c_gradient[2113].im = 0.0;
  c_gradient[2114].re = -1.0;
  c_gradient[2114].im = 0.0;
  c_gradient[2115].re = -1.0;
  c_gradient[2115].im = 0.0;
  c_gradient[2116].re = 0.0;
  c_gradient[2116].im = 0.0;
  c_gradient[2117].re = 0.0;
  c_gradient[2117].im = 0.0;
  c_gradient[2118].re = 0.0;
  c_gradient[2118].im = 0.0;
  c_gradient[2119].re = 0.0;
  c_gradient[2119].im = 0.0;
  c_gradient[2120].re = 0.0;
  c_gradient[2120].im = 0.0;
  c_gradient[2121].re = 0.0;
  c_gradient[2121].im = 0.0;
  c_gradient[2122].re = 0.0;
  c_gradient[2122].im = 0.0;
  c_gradient[2123].re = 0.0;
  c_gradient[2123].im = 0.0;
  c_gradient[2124].re = 0.0;
  c_gradient[2124].im = 0.0;
  c_gradient[2125].re = 0.0;
  c_gradient[2125].im = 0.0;
  c_gradient[2126].re = 0.0;
  c_gradient[2126].im = 0.0;
  c_gradient[2127].re = 0.0;
  c_gradient[2127].im = 0.0;
  c_gradient[2128].re = 0.0;
  c_gradient[2128].im = 0.0;
  c_gradient[2129].re = 0.0;
  c_gradient[2129].im = 0.0;
  c_gradient[2130].re = 0.0;
  c_gradient[2130].im = 0.0;
  c_gradient[2131].re = 0.0;
  c_gradient[2131].im = 0.0;
  c_gradient[2132].re = 0.0;
  c_gradient[2132].im = 0.0;
  c_gradient[2133].re = 0.0;
  c_gradient[2133].im = 0.0;
  c_gradient[2134].re = 0.0;
  c_gradient[2134].im = 0.0;
  c_gradient[2135].re = 0.0;
  c_gradient[2135].im = 0.0;
  c_gradient[2136].re = 0.0;
  c_gradient[2136].im = 0.0;
  c_gradient[2137].re = 0.0;
  c_gradient[2137].im = 0.0;
  c_gradient[2138].re = 0.0;
  c_gradient[2138].im = 0.0;
  c_gradient[2139].re = 0.0;
  c_gradient[2139].im = 0.0;
  c_gradient[2140].re = 0.0;
  c_gradient[2140].im = 0.0;
  c_gradient[2141].re = 0.0;
  c_gradient[2141].im = 0.0;
  c_gradient[2142].re = 0.0;
  c_gradient[2142].im = 0.0;
  c_gradient[2143].re = 0.0;
  c_gradient[2143].im = 0.0;
  c_gradient[2144].re = 0.0;
  c_gradient[2144].im = 0.0;
  c_gradient[2145].re = 0.0;
  c_gradient[2145].im = 0.0;
  c_gradient[2146].re = 0.0;
  c_gradient[2146].im = 0.0;
  c_gradient[2147].re = 0.0;
  c_gradient[2147].im = 0.0;
  c_gradient[2148].re = 0.0;
  c_gradient[2148].im = 0.0;
  c_gradient[2149].re = 0.0;
  c_gradient[2149].im = 0.0;
  c_gradient[2150].re = 0.0;
  c_gradient[2150].im = 0.0;
  c_gradient[2151].re = 0.0;
  c_gradient[2151].im = 0.0;
  c_gradient[2152].re = 0.0;
  c_gradient[2152].im = 0.0;
  c_gradient[2153].re = 0.0;
  c_gradient[2153].im = 0.0;
  c_gradient[2154].re = 0.0;
  c_gradient[2154].im = 0.0;
  c_gradient[2155].re = 0.0;
  c_gradient[2155].im = 0.0;
  c_gradient[2156].re = 0.0;
  c_gradient[2156].im = 0.0;
  c_gradient[2157].re = 0.0;
  c_gradient[2157].im = 0.0;
  c_gradient[2158].re = 0.0;
  c_gradient[2158].im = 0.0;
  c_gradient[2159].re = 0.0;
  c_gradient[2159].im = 0.0;
  c_gradient[2160].re = 0.0;
  c_gradient[2160].im = 0.0;
  c_gradient[2161].re = 0.0;
  c_gradient[2161].im = 0.0;
  c_gradient[2162].re = 0.0;
  c_gradient[2162].im = 0.0;
  c_gradient[2163].re = 0.0;
  c_gradient[2163].im = 0.0;
  c_gradient[2164].re = 0.0;
  c_gradient[2164].im = 0.0;
  c_gradient[2165].re = 0.0;
  c_gradient[2165].im = 0.0;
  c_gradient[2166].re = 0.0;
  c_gradient[2166].im = 0.0;
  c_gradient[2167].re = 0.0;
  c_gradient[2167].im = 0.0;
  c_gradient[2168].re = 0.0;
  c_gradient[2168].im = 0.0;
  c_gradient[2169].re = 0.0;
  c_gradient[2169].im = 0.0;
  c_gradient[2170].re = 0.0;
  c_gradient[2170].im = 0.0;
  c_gradient[2171].re = 0.0;
  c_gradient[2171].im = 0.0;
  c_gradient[2172].re = 0.0;
  c_gradient[2172].im = 0.0;
  for (i = 0; i < 131; i++) {
    c_gradient[i + 2173].re = iv[i];
    c_gradient[i + 2173].im = 0.0;
  }

  memset(&dv[0], 0, 9U * sizeof(double));
  dv[9] = ct->f521;
  dv[10] = 0.0;
  dv[11] = 0.0;
  dv[12] = ct->f557;
  dv[13] = 0.0;
  dv[14] = 0.0;
  dv[15] = ct->f560;
  memset(&dv[16], 0, 11U * sizeof(double));
  dv[27] = ct->f519;
  dv[28] = 0.0;
  dv[29] = 0.0;
  dv[30] = ct->f555;
  dv[31] = 0.0;
  dv[32] = 0.0;
  dv[33] = ct->f559;
  memset(&dv[34], 0, 11U * sizeof(double));
  dv[45] = ct->f518;
  dv[46] = 0.0;
  dv[47] = 0.0;
  dv[48] = ct->f554;
  dv[49] = 0.0;
  dv[50] = 0.0;
  dv[51] = ct->f697;
  memset(&dv[52], 0, 8U * sizeof(double));
  dv[60] = ct->f552;
  dv[61] = 0.0;
  dv[62] = 0.0;
  dv[63] = ct->f517;
  dv[64] = 0.0;
  dv[65] = 0.0;
  dv[66] = ct->f327;
  dv[67] = 0.0;
  dv[68] = 0.0;
  dv[69] = ct->f552;
  dv[70] = 0.0;
  dv[71] = 0.0;
  dv[72] = 0.0;
  dv[73] = 0.0;
  dv[74] = 0.0;
  dv[75] = ct->f4;
  dv[76] = 0.0;
  dv[77] = 0.0;
  dv[78] = 0.0;
  dv[79] = 0.0;
  dv[80] = 0.0;
  dv[81] = ct->f541;
  dv[82] = 0.0;
  dv[83] = 0.0;
  dv[84] = ct->f4;
  dv[85] = 0.0;
  dv[86] = 0.0;
  dv[87] = 0.0;
  dv[88] = 0.0;
  dv[89] = 0.0;
  dv[90] = ct->f3;
  memset(&dv[91], 0, 8U * sizeof(double));
  dv[99] = ct->f3;
  memset(&dv[100], 0, 18U * sizeof(double));
  dv[118] = ct->f521;
  dv[119] = 0.0;
  dv[120] = 0.0;
  dv[121] = ct->f557;
  dv[122] = 0.0;
  dv[123] = 0.0;
  dv[124] = ct->f560;
  memset(&dv[125], 0, 11U * sizeof(double));
  dv[136] = ct->f519;
  dv[137] = 0.0;
  dv[138] = 0.0;
  dv[139] = ct->f555;
  dv[140] = 0.0;
  dv[141] = 0.0;
  dv[142] = ct->f559;
  memset(&dv[143], 0, 11U * sizeof(double));
  dv[154] = ct->f518;
  dv[155] = 0.0;
  dv[156] = 0.0;
  dv[157] = ct->f554;
  dv[158] = 0.0;
  dv[159] = 0.0;
  dv[160] = ct->f697;
  memset(&dv[161], 0, 8U * sizeof(double));
  dv[169] = ct->f552;
  dv[170] = 0.0;
  dv[171] = 0.0;
  dv[172] = ct->f517;
  dv[173] = 0.0;
  dv[174] = 0.0;
  dv[175] = ct->f327;
  dv[176] = 0.0;
  dv[177] = 0.0;
  dv[178] = ct->f552;
  dv[179] = 0.0;
  dv[180] = 0.0;
  dv[181] = 0.0;
  dv[182] = 0.0;
  dv[183] = 0.0;
  dv[184] = ct->f4;
  dv[185] = 0.0;
  dv[186] = 0.0;
  dv[187] = 0.0;
  dv[188] = 0.0;
  dv[189] = 0.0;
  dv[190] = ct->f541;
  dv[191] = 0.0;
  dv[192] = 0.0;
  dv[193] = ct->f4;
  dv[194] = 0.0;
  dv[195] = 0.0;
  dv[196] = 0.0;
  dv[197] = 0.0;
  dv[198] = 0.0;
  dv[199] = ct->f3;
  memset(&dv[200], 0, 8U * sizeof(double));
  dv[208] = ct->f3;
  memset(&dv[209], 0, 18U * sizeof(double));
  dv[227] = ct->f521;
  dv[228] = 0.0;
  dv[229] = 0.0;
  dv[230] = ct->f557;
  dv[231] = 0.0;
  dv[232] = 0.0;
  dv[233] = ct->f560;
  memset(&dv[234], 0, 11U * sizeof(double));
  dv[245] = ct->f519;
  dv[246] = 0.0;
  dv[247] = 0.0;
  dv[248] = ct->f555;
  dv[249] = 0.0;
  dv[250] = 0.0;
  dv[251] = ct->f559;
  memset(&dv[252], 0, 11U * sizeof(double));
  dv[263] = ct->f518;
  dv[264] = 0.0;
  dv[265] = 0.0;
  dv[266] = ct->f554;
  dv[267] = 0.0;
  dv[268] = 0.0;
  dv[269] = ct->f697;
  memset(&dv[270], 0, 8U * sizeof(double));
  dv[278] = ct->f552;
  dv[279] = 0.0;
  dv[280] = 0.0;
  dv[281] = ct->f517;
  dv[282] = 0.0;
  dv[283] = 0.0;
  dv[284] = ct->f327;
  dv[285] = 0.0;
  dv[286] = 0.0;
  dv[287] = ct->f552;
  dv[288] = 0.0;
  dv[289] = 0.0;
  dv[290] = 0.0;
  dv[291] = 0.0;
  dv[292] = 0.0;
  dv[293] = ct->f4;
  dv[294] = 0.0;
  dv[295] = 0.0;
  dv[296] = 0.0;
  dv[297] = 0.0;
  dv[298] = 0.0;
  dv[299] = ct->f541;
  dv[300] = 0.0;
  dv[301] = 0.0;
  dv[302] = ct->f4;
  dv[303] = 0.0;
  dv[304] = 0.0;
  dv[305] = 0.0;
  dv[306] = 0.0;
  dv[307] = 0.0;
  dv[308] = ct->f3;
  memset(&dv[309], 0, 8U * sizeof(double));
  dv[317] = ct->f3;
  dv[318] = 0.0;
  dv[319] = 0.0;
  dv[320] = 0.0;
  dv[321] = 0.0;
  dv[322] = 0.0;
  dv[323] = 0.0;
  ceq_gradient_size[0] = 18;
  ceq_gradient_size[1] = 18;
  memcpy(&ceq_gradient_data[0], &dv[0], 324U * sizeof(double));
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/*
 * COMPUTE_CONSTRAINTS_AND_CONSTRAINTS_GRADIENT_W_SHIP_COEFF
 *     [C,CEQ,C_GRADIENT,CEQ_GRADIENT] = COMPUTE_CONSTRAINTS_AND_CONSTRAINTS_GRADIENT_W_SHIP_COEFF(Ax_0_UAV,Ax_min_control_UAV,Ax_max_control_UAV,Ay_0_UAV,Ay_min_control_UAV,Ay_max_control_UAV,Az_0_UAV,Az_min_control_UAV,Az_max_control_UAV,Vx_0_UAV,Vx_min_control_UAV,Vx_max_control_UAV,Vy_0_UAV,Vy_min_control_UAV,Vy_max_control_UAV,Vz_0_UAV,Vz_min_control_UAV,Vz_max_control_UAV,X_0_UAV,Y_0_UAV,Z_0_UAV,ACC_GAIN,coeff_1_UAV,coeff_2_UAV,coeff_3_UAV,coeff_4_UAV,coeff_5_UAV,coeff_6_UAV,coeff_7_UAV,coeff_8_UAV,coeff_9_UAV,coeff_10_UAV,coeff_11_UAV,coeff_12_UAV,coeff_13_UAV,coeff_14_UAV,coeff_15_UAV,coeff_16_UAV,coeff_17_UAV,coeff_18_UAV,COEFF_1_SHIP_PREDICTION,COEFF_2_SHIP_PREDICTION,COEFF_3_SHIP_PREDICTION,COEFF_4_SHIP_PREDICTION,COEFF_5_SHIP_PREDICTION,COEFF_6_SHIP_PREDICTION,COEFF_7_SHIP_PREDICTION,COEFF_8_SHIP_PREDICTION,COEFF_9_SHIP_PREDICTION,COEFF_10_SHIP_PREDICTION,COEFF_11_SHIP_PREDICTION,COEFF_12_SHIP_PREDICTION,COEFF_13_SHIP_PREDICTION,COEFF_14_SHIP_PREDICTION,COEFF_15_SHIP_PREDICTION,COEFF_16_SHIP_PREDICTION,COEFF_17_SHIP_PREDICTION,COEFF_18_SHIP_PREDICTION,POS_GAIN,SPEED_GAIN,T_2_ARRAY,T_3_ARRAY,T_4_ARRAY,T_5_ARRAY,T_6_ARRAY,T_7_ARRAY,T_8_ARRAY,T_9_ARRAY,T_LANDING)
 *
 * Arguments    : double Ax_0_UAV
 *                double Ax_min_control_UAV
 *                double Ax_max_control_UAV
 *                double Ay_0_UAV
 *                double Ay_min_control_UAV
 *                double Ay_max_control_UAV
 *                double Az_0_UAV
 *                double Az_min_control_UAV
 *                double Az_max_control_UAV
 *                double Vx_0_UAV
 *                double Vx_min_control_UAV
 *                double Vx_max_control_UAV
 *                double Vy_0_UAV
 *                double Vy_min_control_UAV
 *                double Vy_max_control_UAV
 *                double Vz_0_UAV
 *                double Vz_min_control_UAV
 *                double Vz_max_control_UAV
 *                double X_0_UAV
 *                double Y_0_UAV
 *                double Z_0_UAV
 *                double acc_gain
 *                double coeff_1_UAV
 *                double coeff_2_UAV
 *                double coeff_3_UAV
 *                double coeff_4_UAV
 *                double coeff_5_UAV
 *                double coeff_6_UAV
 *                double coeff_7_UAV
 *                double coeff_8_UAV
 *                double coeff_9_UAV
 *                double coeff_10_UAV
 *                double coeff_11_UAV
 *                double coeff_12_UAV
 *                double coeff_13_UAV
 *                double coeff_14_UAV
 *                double coeff_15_UAV
 *                double coeff_16_UAV
 *                double coeff_17_UAV
 *                double coeff_18_UAV
 *                double coeff_1_ship_prediction
 *                double coeff_2_ship_prediction
 *                double coeff_3_ship_prediction
 *                double coeff_4_ship_prediction
 *                double coeff_5_ship_prediction
 *                double coeff_6_ship_prediction
 *                double coeff_7_ship_prediction
 *                double coeff_8_ship_prediction
 *                double coeff_9_ship_prediction
 *                double coeff_10_ship_prediction
 *                double coeff_11_ship_prediction
 *                double coeff_12_ship_prediction
 *                double coeff_13_ship_prediction
 *                double coeff_14_ship_prediction
 *                double coeff_15_ship_prediction
 *                double coeff_16_ship_prediction
 *                double coeff_17_ship_prediction
 *                double coeff_18_ship_prediction
 *                double pos_gain
 *                double speed_gain
 *                double t_2_array
 *                double t_3_array
 *                double t_4_array
 *                double t_5_array
 *                double t_6_array
 *                double t_7_array
 *                double t_8_array
 *                double t_9_array
 *                double t_landing
 *                double c_data[]
 *                int c_size[2]
 *                double ceq_data[]
 *                int ceq_size[2]
 *                creal_T c_gradient_data[]
 *                int c_gradient_size[2]
 *                double ceq_gradient_data[]
 *                int ceq_gradient_size[2]
 * Return Type  : void
 */
void c_compute_constraints_and_const(double Ax_0_UAV, double Ax_min_control_UAV,
  double Ax_max_control_UAV, double Ay_0_UAV, double Ay_min_control_UAV, double
  Ay_max_control_UAV, double Az_0_UAV, double Az_min_control_UAV, double
  Az_max_control_UAV, double Vx_0_UAV, double Vx_min_control_UAV, double
  Vx_max_control_UAV, double Vy_0_UAV, double Vy_min_control_UAV, double
  Vy_max_control_UAV, double Vz_0_UAV, double Vz_min_control_UAV, double
  Vz_max_control_UAV, double X_0_UAV, double Y_0_UAV, double Z_0_UAV, double
  acc_gain, double coeff_1_UAV, double coeff_2_UAV, double coeff_3_UAV, double
  coeff_4_UAV, double coeff_5_UAV, double coeff_6_UAV, double coeff_7_UAV,
  double coeff_8_UAV, double coeff_9_UAV, double coeff_10_UAV, double
  coeff_11_UAV, double coeff_12_UAV, double coeff_13_UAV, double coeff_14_UAV,
  double coeff_15_UAV, double coeff_16_UAV, double coeff_17_UAV, double
  coeff_18_UAV, double coeff_1_ship_prediction, double coeff_2_ship_prediction,
  double coeff_3_ship_prediction, double coeff_4_ship_prediction, double
  coeff_5_ship_prediction, double coeff_6_ship_prediction, double
  coeff_7_ship_prediction, double coeff_8_ship_prediction, double
  coeff_9_ship_prediction, double coeff_10_ship_prediction, double
  coeff_11_ship_prediction, double coeff_12_ship_prediction, double
  coeff_13_ship_prediction, double coeff_14_ship_prediction, double
  coeff_15_ship_prediction, double coeff_16_ship_prediction, double
  coeff_17_ship_prediction, double coeff_18_ship_prediction, double pos_gain,
  double speed_gain, double t_2_array, double t_3_array, double t_4_array,
  double t_5_array, double t_6_array, double t_7_array, double t_8_array, double
  t_9_array, double t_landing, double c_data[], int c_size[2], double ceq_data[],
  int ceq_size[2], creal_T c_gradient_data[], int c_gradient_size[2], double
  ceq_gradient_data[], int ceq_gradient_size[2])
{
  cell_14 expl_temp;
  double t102;
  double t105;
  double t109;
  double t11;
  double t113;
  double t116;
  double t120;
  double t134;
  double t137;
  double t150;
  double t151;
  double t152;
  double t153;
  double t154;
  double t155;
  double t156;
  double t157;
  double t158;
  double t159;
  double t160;
  double t161;
  double t162;
  double t163;
  double t164;
  double t165;
  double t166;
  double t167;
  double t168;
  double t169;
  double t17;
  double t170;
  double t171;
  double t172;
  double t173;
  double t174;
  double t175;
  double t176;
  double t181_re;
  double t182;
  double t183;
  double t184;
  double t185;
  double t186;
  double t187;
  double t188;
  double t189;
  double t19;
  double t190;
  double t191;
  double t192;
  double t193;
  double t194;
  double t195;
  double t196;
  double t197;
  double t198;
  double t199;
  double t2;
  double t200_im;
  double t200_re;
  double t200_re_tmp;
  double t201_im;
  double t201_re;
  double t201_re_tmp;
  double t202_im;
  double t202_re;
  double t202_re_tmp;
  double t203_im;
  double t203_re;
  double t203_re_tmp;
  double t204_im;
  double t204_re;
  double t204_re_tmp;
  double t205_im;
  double t205_re;
  double t205_re_tmp;
  double t206_im;
  double t206_re;
  double t206_re_tmp;
  double t207_im;
  double t207_re;
  double t207_re_tmp;
  double t208_im;
  double t208_re;
  double t208_re_tmp;
  double t21;
  double t23;
  double t25;
  double t26;
  double t27;
  double t28;
  double t29;
  double t30;
  double t31;
  double t32;
  double t33;
  double t34;
  double t35;
  double t36;
  double t38;
  double t39;
  double t4;
  double t40;
  double t414_im;
  double t414_re;
  double t415_im;
  double t415_re;
  double t415_re_tmp;
  double t416_im;
  double t416_re;
  double t417_im;
  double t417_re;
  double t417_re_tmp;
  double t418_im;
  double t418_re;
  double t418_re_tmp;
  double t419_im;
  double t419_re;
  double t42;
  double t420_im;
  double t420_re;
  double t420_re_tmp;
  double t421_im;
  double t421_re;
  double t421_re_tmp;
  double t422_im;
  double t422_re;
  double t423_im;
  double t423_re;
  double t423_re_tmp;
  double t424_im;
  double t424_re;
  double t424_re_tmp;
  double t425_im;
  double t425_re;
  double t426_im;
  double t426_re;
  double t426_re_tmp;
  double t427_im;
  double t427_re;
  double t427_re_tmp;
  double t428_im;
  double t428_re;
  double t429_im;
  double t429_re;
  double t429_re_tmp;
  double t43;
  double t430_im;
  double t430_re;
  double t430_re_tmp;
  double t431_im;
  double t431_re;
  double t432_im;
  double t432_re;
  double t432_re_tmp;
  double t433_im;
  double t433_re;
  double t433_re_tmp;
  double t434_im;
  double t434_re;
  double t435_im;
  double t435_re;
  double t435_re_tmp;
  double t436_im;
  double t436_re;
  double t436_re_tmp;
  double t437_im;
  double t437_re;
  double t437_re_tmp;
  double t438_im;
  double t438_re;
  double t439_im;
  double t439_re;
  double t439_re_tmp;
  double t44;
  double t440_im;
  double t440_re;
  double t440_re_tmp;
  double t46;
  double t468;
  double t47;
  double t48;
  double t5;
  double t50;
  double t51;
  double t52;
  double t54;
  double t55;
  double t56;
  double t58;
  double t59;
  double t6;
  double t61;
  double t63;
  double t64;
  double t65;
  double t7;
  double t72;
  double t75;
  double t78;
  double t8;
  double t81;
  double t85;
  double t89;
  double t9;
  double t93;
  double t95;
  double t98;
  double t99;

  /*     This function was generated by the Symbolic Math Toolbox version 23.2. */
  /*     12-Jun-2024 13:42:59 */
  t2 = pos_gain * t_landing;
  t4 = coeff_4_UAV * 2.0;
  t5 = coeff_5_UAV * 2.0;
  t6 = coeff_10_UAV * 2.0;
  t7 = coeff_11_UAV * 2.0;
  t8 = coeff_16_UAV * 2.0;
  t9 = coeff_5_UAV * coeff_5_UAV;
  t11 = coeff_11_UAV * coeff_11_UAV;
  t17 = t_2_array * 6.0;
  t19 = t_3_array * 6.0;
  t21 = t_4_array * 6.0;
  t23 = t_5_array * 6.0;
  t25 = t_6_array * 6.0;
  t26 = t_7_array * 6.0;
  t27 = t_8_array * 6.0;
  t28 = t_9_array * 6.0;
  t29 = t_landing * 2.0;
  t30 = t_landing * 6.0;
  t31 = t_2_array * t_2_array;
  t32 = rt_powd_snf(t_2_array, 3.0);
  t33 = t_3_array * t_3_array;
  t35 = rt_powd_snf(t_3_array, 3.0);
  t36 = t_4_array * t_4_array;
  t39 = rt_powd_snf(t_4_array, 3.0);
  t40 = t_5_array * t_5_array;
  t43 = rt_powd_snf(t_5_array, 3.0);
  t44 = t_6_array * t_6_array;
  t47 = rt_powd_snf(t_6_array, 3.0);
  t48 = t_7_array * t_7_array;
  t51 = rt_powd_snf(t_7_array, 3.0);
  t52 = t_8_array * t_8_array;
  t55 = rt_powd_snf(t_8_array, 3.0);
  t56 = t_9_array * t_9_array;
  t59 = rt_powd_snf(t_9_array, 3.0);
  t63 = t_landing * t_landing;
  t64 = rt_powd_snf(t_landing, 3.0);
  t181_re = coeff_11_UAV * 0.0;
  t200_re_tmp = coeff_10_UAV * t_2_array;
  t200_re = t200_re_tmp * 0.0;
  t200_im = t200_re_tmp * 2.0;
  t201_re_tmp = coeff_10_UAV * t_3_array;
  t201_re = t201_re_tmp * 0.0;
  t201_im = t201_re_tmp * 2.0;
  t202_re_tmp = coeff_10_UAV * t_4_array;
  t202_re = t202_re_tmp * 0.0;
  t202_im = t202_re_tmp * 2.0;
  t203_re_tmp = coeff_10_UAV * t_5_array;
  t203_re = t203_re_tmp * 0.0;
  t203_im = t203_re_tmp * 2.0;
  t204_re_tmp = coeff_10_UAV * t_6_array;
  t204_re = t204_re_tmp * 0.0;
  t204_im = t204_re_tmp * 2.0;
  t205_re_tmp = coeff_10_UAV * t_7_array;
  t205_re = t205_re_tmp * 0.0;
  t205_im = t205_re_tmp * 2.0;
  t206_re_tmp = coeff_10_UAV * t_8_array;
  t206_re = t206_re_tmp * 0.0;
  t206_im = t206_re_tmp * 2.0;
  t207_re_tmp = coeff_10_UAV * t_9_array;
  t207_re = t207_re_tmp * 0.0;
  t207_im = t207_re_tmp * 2.0;
  t208_re_tmp = coeff_10_UAV * t_landing;
  t208_re = t208_re_tmp * 0.0;
  t208_im = t208_re_tmp * 2.0;
  t34 = t31 * t31;
  t38 = t33 * t33;
  t42 = t36 * t36;
  t46 = t40 * t40;
  t50 = t44 * t44;
  t54 = t48 * t48;
  t58 = t52 * t52;
  t61 = t56 * t56;
  t65 = t63 * t63;
  t72 = coeff_3_UAV * t17;
  t75 = coeff_3_UAV * t19;
  t78 = coeff_3_UAV * t21;
  t81 = coeff_3_UAV * t23;
  t85 = coeff_3_UAV * t25;
  t89 = coeff_3_UAV * t26;
  t93 = coeff_3_UAV * t27;
  t95 = coeff_9_UAV * t17;
  t98 = coeff_3_UAV * t28;
  t99 = coeff_9_UAV * t19;
  t102 = coeff_9_UAV * t21;
  t105 = coeff_9_UAV * t23;
  t109 = coeff_9_UAV * t25;
  t113 = coeff_9_UAV * t26;
  t116 = coeff_9_UAV * t27;
  t120 = coeff_9_UAV * t28;
  t134 = coeff_3_UAV * t30;
  t137 = coeff_9_UAV * t30;
  t150 = t31 * 3.0;
  t151 = t33 * 3.0;
  t152 = t32 * 4.0;
  t153 = t36 * 3.0;
  t154 = t35 * 4.0;
  t155 = t40 * 3.0;
  t157 = t39 * 4.0;
  t158 = t44 * 3.0;
  t160 = t43 * 4.0;
  t161 = t48 * 3.0;
  t163 = t47 * 4.0;
  t164 = t52 * 3.0;
  t166 = t51 * 4.0;
  t167 = t56 * 3.0;
  t169 = t55 * 4.0;
  t171 = t59 * 4.0;
  t174 = t63 * 3.0;
  t175 = t64 * 4.0;
  t182 = t31 * 12.0;
  t183 = t33 * 12.0;
  t184 = t36 * 12.0;
  t185 = t40 * 12.0;
  t186 = t44 * 12.0;
  t187 = t48 * 12.0;
  t188 = t52 * 12.0;
  t189 = t56 * 12.0;
  t190 = t32 * 20.0;
  t191 = t35 * 20.0;
  t192 = t39 * 20.0;
  t193 = t43 * 20.0;
  t194 = t47 * 20.0;
  t195 = t51 * 20.0;
  t196 = t55 * 20.0;
  t197 = t59 * 20.0;
  t198 = t63 * 12.0;
  t199 = t64 * 20.0;
  t438_im = coeff_9_UAV * t31;
  t414_re = t438_im * 0.0;
  t414_im = t438_im * 3.0;
  t415_re_tmp = coeff_8_UAV * t32;
  t415_re = t415_re_tmp * 0.0;
  t415_im = t415_re_tmp * 4.0;
  t438_im = coeff_9_UAV * t33;
  t416_re = t438_im * 0.0;
  t416_im = t438_im * 3.0;
  t418_re_tmp = coeff_8_UAV * t35;
  t418_re = t418_re_tmp * 0.0;
  t418_im = t418_re_tmp * 4.0;
  t438_im = coeff_9_UAV * t36;
  t419_re = t438_im * 0.0;
  t419_im = t438_im * 3.0;
  t421_re_tmp = coeff_8_UAV * t39;
  t421_re = t421_re_tmp * 0.0;
  t421_im = t421_re_tmp * 4.0;
  t438_im = coeff_9_UAV * t40;
  t422_re = t438_im * 0.0;
  t422_im = t438_im * 3.0;
  t424_re_tmp = coeff_8_UAV * t43;
  t424_re = t424_re_tmp * 0.0;
  t424_im = t424_re_tmp * 4.0;
  t438_im = coeff_9_UAV * t44;
  t425_re = t438_im * 0.0;
  t425_im = t438_im * 3.0;
  t427_re_tmp = coeff_8_UAV * t47;
  t427_re = t427_re_tmp * 0.0;
  t427_im = t427_re_tmp * 4.0;
  t438_im = coeff_9_UAV * t48;
  t428_re = t438_im * 0.0;
  t428_im = t438_im * 3.0;
  t430_re_tmp = coeff_8_UAV * t51;
  t430_re = t430_re_tmp * 0.0;
  t430_im = t430_re_tmp * 4.0;
  t438_im = coeff_9_UAV * t52;
  t431_re = t438_im * 0.0;
  t431_im = t438_im * 3.0;
  t433_re_tmp = coeff_8_UAV * t55;
  t433_re = t433_re_tmp * 0.0;
  t433_im = t433_re_tmp * 4.0;
  t438_im = coeff_9_UAV * t56;
  t434_re = t438_im * 0.0;
  t434_im = t438_im * 3.0;
  t436_re_tmp = coeff_8_UAV * t59;
  t436_re = t436_re_tmp * 0.0;
  t436_im = t436_re_tmp * 4.0;
  t438_im = coeff_9_UAV * t63;
  t438_re = t438_im * 0.0;
  t438_im *= 3.0;
  t439_re_tmp = coeff_8_UAV * t64;
  t439_re = t439_re_tmp * 0.0;
  t439_im = t439_re_tmp * 4.0;
  t156 = t34 * 5.0;
  t159 = t38 * 5.0;
  t162 = t42 * 5.0;
  t165 = t46 * 5.0;
  t168 = t50 * 5.0;
  t170 = t54 * 5.0;
  t172 = t58 * 5.0;
  t173 = t61 * 5.0;
  t176 = t65 * 5.0;
  t417_re_tmp = coeff_7_UAV * t34;
  t417_re = t417_re_tmp * 0.0;
  t417_im = t417_re_tmp * 5.0;
  t420_re_tmp = coeff_7_UAV * t38;
  t420_re = t420_re_tmp * 0.0;
  t420_im = t420_re_tmp * 5.0;
  t423_re_tmp = coeff_7_UAV * t42;
  t423_re = t423_re_tmp * 0.0;
  t423_im = t423_re_tmp * 5.0;
  t426_re_tmp = coeff_7_UAV * t46;
  t426_re = t426_re_tmp * 0.0;
  t426_im = t426_re_tmp * 5.0;
  t429_re_tmp = coeff_7_UAV * t50;
  t429_re = t429_re_tmp * 0.0;
  t429_im = t429_re_tmp * 5.0;
  t432_re_tmp = coeff_7_UAV * t54;
  t432_re = t432_re_tmp * 0.0;
  t432_im = t432_re_tmp * 5.0;
  t435_re_tmp = coeff_7_UAV * t58;
  t435_re = t435_re_tmp * 0.0;
  t435_im = t435_re_tmp * 5.0;
  t437_re_tmp = coeff_7_UAV * t61;
  t437_re = t437_re_tmp * 0.0;
  t437_im = t437_re_tmp * 5.0;
  t440_re_tmp = coeff_7_UAV * t65;
  t440_re = t440_re_tmp * 0.0;
  t440_im = t440_re_tmp * 5.0;
  t468 = 1.0 / sqrt(t9 + t11);
  expl_temp.f491 = t_landing;
  expl_temp.f490 = t_9_array;
  expl_temp.f489 = t_8_array;
  expl_temp.f488 = t_7_array;
  expl_temp.f487 = t_6_array;
  expl_temp.f486 = t_5_array;
  expl_temp.f485 = t_4_array;
  expl_temp.f484 = t_3_array;
  expl_temp.f483 = t_2_array;
  expl_temp.f482 = t6 * t_5_array;
  expl_temp.f481 = t201_re_tmp * 4.0;
  expl_temp.f480 = coeff_4_UAV * t_9_array * 4.0;
  expl_temp.f479 = t6 * t_4_array;
  expl_temp.f478 = t200_re_tmp * 4.0;
  expl_temp.f477 = coeff_4_UAV * t_8_array * 4.0;
  expl_temp.f476 = t9;
  expl_temp.f475 = t6 * t_3_array;
  expl_temp.f474 = t4 * t_9_array;
  expl_temp.f473 = coeff_4_UAV * t_7_array * 4.0;
  expl_temp.f472 = t6 * t_2_array;
  expl_temp.f471 = t4 * t_8_array;
  expl_temp.f470 = coeff_4_UAV * t_6_array * 4.0;
  expl_temp.f469 = t4 * t_7_array;
  expl_temp.f468 = t8;
  expl_temp.f467 = t4 * t_6_array;
  expl_temp.f466 = t4 * t_5_array;
  expl_temp.f465 = t4 * t_4_array;
  expl_temp.f464 = t7;
  expl_temp.f463 = t4 * t_3_array;
  expl_temp.f462 = t4 * t_2_array;
  expl_temp.f461 = acc_gain * t30;
  expl_temp.f460 = rt_powd_snf(t_landing, 5.0);
  expl_temp.f459 = t65;
  expl_temp.f458 = t64;
  expl_temp.f457 = t63;
  expl_temp.f456 = rt_powd_snf(t_9_array, 5.0);
  expl_temp.f455 = t61;
  expl_temp.f454 = rt_powd_snf(t_8_array, 5.0);
  expl_temp.f453 = t6;
  expl_temp.f452 = t59;
  expl_temp.f451 = t58;
  expl_temp.f450 = rt_powd_snf(t_7_array, 5.0);
  expl_temp.f449 = t56;
  expl_temp.f448 = (((t5 + coeff_4_UAV * t_5_array * 4.0) + t81 * t_5_array) +
                    coeff_2_UAV * t43 * 8.0) + coeff_1_UAV * t46 * 10.0;
  expl_temp.f447 = (((t5 + coeff_4_UAV * t_4_array * 4.0) + t78 * t_4_array) +
                    coeff_2_UAV * t39 * 8.0) + coeff_1_UAV * t42 * 10.0;
  expl_temp.f446 = (((t5 + coeff_4_UAV * t_3_array * 4.0) + t75 * t_3_array) +
                    coeff_2_UAV * t35 * 8.0) + coeff_1_UAV * t38 * 10.0;
  expl_temp.f445 = (((t5 + coeff_4_UAV * t_2_array * 4.0) + t72 * t_2_array) +
                    coeff_2_UAV * t32 * 8.0) + coeff_1_UAV * t34 * 10.0;
  expl_temp.f444 = t55;
  expl_temp.f443 = t54;
  expl_temp.f442 = rt_powd_snf(t_6_array, 5.0);
  expl_temp.f441 = t52;
  expl_temp.f440 = ((t6 + t137) + coeff_8_UAV * t198) + coeff_7_UAV * t199;
  expl_temp.f439 = ((t4 + t134) + coeff_2_UAV * t198) + coeff_1_UAV * t199;
  expl_temp.f438 = ((t6 + t120) + coeff_8_UAV * t189) + coeff_7_UAV * t197;
  expl_temp.f437 = ((t6 + t116) + coeff_8_UAV * t188) + coeff_7_UAV * t196;
  expl_temp.f436 = t51;
  expl_temp.f435 = ((t6 + t113) + coeff_8_UAV * t187) + coeff_7_UAV * t195;
  expl_temp.f434 = ((t6 + t109) + coeff_8_UAV * t186) + coeff_7_UAV * t194;
  expl_temp.f433 = ((t6 + t105) + coeff_8_UAV * t185) + coeff_7_UAV * t193;
  expl_temp.f432 = ((t6 + t102) + coeff_8_UAV * t184) + coeff_7_UAV * t192;
  expl_temp.f431 = ((t6 + t99) + coeff_8_UAV * t183) + coeff_7_UAV * t191;
  expl_temp.f430 = ((t6 + t95) + coeff_8_UAV * t182) + coeff_7_UAV * t190;
  expl_temp.f429 = ((t4 + t98) + coeff_2_UAV * t189) + coeff_1_UAV * t197;
  expl_temp.f428 = ((t4 + t93) + coeff_2_UAV * t188) + coeff_1_UAV * t196;
  expl_temp.f427 = ((t4 + t89) + coeff_2_UAV * t187) + coeff_1_UAV * t195;
  expl_temp.f426 = ((t4 + t85) + coeff_2_UAV * t186) + coeff_1_UAV * t194;
  expl_temp.f425 = t50;
  expl_temp.f424 = t5;
  expl_temp.f423 = ((t4 + t81) + coeff_2_UAV * t185) + coeff_1_UAV * t193;
  expl_temp.f422 = ((t4 + t78) + coeff_2_UAV * t184) + coeff_1_UAV * t192;
  expl_temp.f421 = ((t4 + t75) + coeff_2_UAV * t183) + coeff_1_UAV * t191;
  expl_temp.f420 = ((t4 + t72) + coeff_2_UAV * t182) + coeff_1_UAV * t190;
  expl_temp.f419 = rt_powd_snf(t_5_array, 5.0);
  expl_temp.f418 = t11 * t468;
  expl_temp.f417 = t48;
  expl_temp.f416 = t9 * t468;
  expl_temp.f415 = coeff_11_UAV * t468 * -2.0;
  expl_temp.f414 = coeff_10_UAV * t468 * -2.0;
  expl_temp.f413 = coeff_5_UAV * t468 * -2.0;
  expl_temp.f412 = coeff_4_UAV * t468 * -2.0;
  expl_temp.f411 = t7 * t468;
  expl_temp.f410 = t6 * t468;
  expl_temp.f409 = t5 * t468;
  expl_temp.f408 = t4 * t468;
  expl_temp.f407 = t47;
  expl_temp.f406 = rt_powd_snf(t468, 3.0);
  expl_temp.f405.re = -t440_re;
  expl_temp.f405.im = -t440_im;
  expl_temp.f404.re = -t439_re;
  expl_temp.f404.im = -t439_im;
  expl_temp.f403.re = -t438_re;
  expl_temp.f403.im = -t438_im;
  expl_temp.f402.re = -t437_re;
  expl_temp.f402.im = -t437_im;
  expl_temp.f401.re = -t436_re;
  expl_temp.f401.im = -t436_im;
  expl_temp.f400.re = -t435_re;
  expl_temp.f400.im = -t435_im;
  expl_temp.f399.re = -t434_re;
  expl_temp.f399.im = -t434_im;
  expl_temp.f398.re = -t433_re;
  expl_temp.f398.im = -t433_im;
  expl_temp.f397 = t46;
  expl_temp.f396.re = -t432_re;
  expl_temp.f396.im = -t432_im;
  expl_temp.f395.re = -t431_re;
  expl_temp.f395.im = -t431_im;
  expl_temp.f394.re = -t430_re;
  expl_temp.f394.im = -t430_im;
  expl_temp.f393.re = -t429_re;
  expl_temp.f393.im = -t429_im;
  expl_temp.f392.re = -t428_re;
  expl_temp.f392.im = -t428_im;
  expl_temp.f391.re = -t427_re;
  expl_temp.f391.im = -t427_im;
  expl_temp.f390.re = -t426_re;
  expl_temp.f390.im = -t426_im;
  expl_temp.f389.re = -t425_re;
  expl_temp.f389.im = -t425_im;
  expl_temp.f388.re = -t424_re;
  expl_temp.f388.im = -t424_im;
  expl_temp.f387.re = -t423_re;
  expl_temp.f387.im = -t423_im;
  expl_temp.f386 = rt_powd_snf(t_4_array, 5.0);
  expl_temp.f385.re = -t422_re;
  expl_temp.f385.im = -t422_im;
  expl_temp.f384.re = -t421_re;
  expl_temp.f384.im = -t421_im;
  expl_temp.f383.re = -t420_re;
  expl_temp.f383.im = -t420_im;
  expl_temp.f382.re = -t419_re;
  expl_temp.f382.im = -t419_im;
  expl_temp.f381.re = -t418_re;
  expl_temp.f381.im = -t418_im;
  expl_temp.f380.re = -t417_re;
  expl_temp.f380.im = -t417_im;
  expl_temp.f379.re = -t416_re;
  expl_temp.f379.im = -t416_im;
  expl_temp.f378.re = -t415_re;
  expl_temp.f378.im = -t415_im;
  expl_temp.f377.re = -t414_re;
  expl_temp.f377.im = -t414_im;
  expl_temp.f376.re = t440_re;
  expl_temp.f376.im = t440_im;
  expl_temp.f375 = t44;
  expl_temp.f374.re = t439_re;
  expl_temp.f374.im = t439_im;
  expl_temp.f373.re = t438_re;
  expl_temp.f373.im = t438_im;
  expl_temp.f372.re = t437_re;
  expl_temp.f372.im = t437_im;
  expl_temp.f371.re = t436_re;
  expl_temp.f371.im = t436_im;
  expl_temp.f370.re = t435_re;
  expl_temp.f370.im = t435_im;
  expl_temp.f369.re = t434_re;
  expl_temp.f369.im = t434_im;
  expl_temp.f368.re = t433_re;
  expl_temp.f368.im = t433_im;
  expl_temp.f367.re = t432_re;
  expl_temp.f367.im = t432_im;
  expl_temp.f366.re = t431_re;
  expl_temp.f366.im = t431_im;
  expl_temp.f365.re = t430_re;
  expl_temp.f365.im = t430_im;
  expl_temp.f364 = t43;
  expl_temp.f363.re = t429_re;
  expl_temp.f363.im = t429_im;
  expl_temp.f362.re = t428_re;
  expl_temp.f362.im = t428_im;
  expl_temp.f361.re = t427_re;
  expl_temp.f361.im = t427_im;
  expl_temp.f360.re = t426_re;
  expl_temp.f360.im = t426_im;
  expl_temp.f359.re = t425_re;
  expl_temp.f359.im = t425_im;
  expl_temp.f358.re = t424_re;
  expl_temp.f358.im = t424_im;
  expl_temp.f357.re = t423_re;
  expl_temp.f357.im = t423_im;
  expl_temp.f356.re = t422_re;
  expl_temp.f356.im = t422_im;
  expl_temp.f355.re = t421_re;
  expl_temp.f355.im = t421_im;
  expl_temp.f354.re = t420_re;
  expl_temp.f354.im = t420_im;
  expl_temp.f353 = t42;
  expl_temp.f352.re = t419_re;
  expl_temp.f352.im = t419_im;
  expl_temp.f351.re = t418_re;
  expl_temp.f351.im = t418_im;
  expl_temp.f350.re = t417_re;
  expl_temp.f350.im = t417_im;
  expl_temp.f349.re = t416_re;
  expl_temp.f349.im = t416_im;
  expl_temp.f348.re = t415_re;
  expl_temp.f348.im = t415_im;
  expl_temp.f347.re = t414_re;
  expl_temp.f347.im = t414_im;
  expl_temp.f346 = coeff_13_UAV * t199;
  expl_temp.f345 = coeff_14_UAV * t198;
  expl_temp.f344 = rt_powd_snf(t_3_array, 5.0);
  expl_temp.f343 = t440_re_tmp * 10.0;
  expl_temp.f342 = t439_re_tmp * 8.0;
  expl_temp.f341 = coeff_1_UAV * t65 * 10.0;
  expl_temp.f340 = coeff_2_UAV * t64 * 8.0;
  expl_temp.f339 = coeff_13_UAV * t197;
  expl_temp.f338 = coeff_13_UAV * t196;
  expl_temp.f337 = coeff_13_UAV * t195;
  expl_temp.f336 = t40;
  expl_temp.f335 = t4;
  expl_temp.f334 = coeff_13_UAV * t194;
  expl_temp.f333 = coeff_13_UAV * t193;
  expl_temp.f332 = coeff_13_UAV * t192;
  expl_temp.f331 = coeff_13_UAV * t191;
  expl_temp.f330 = coeff_13_UAV * t190;
  expl_temp.f329 = coeff_14_UAV * t189;
  expl_temp.f328 = coeff_14_UAV * t188;
  expl_temp.f327 = t39;
  expl_temp.f326 = coeff_14_UAV * t187;
  expl_temp.f325 = coeff_14_UAV * t186;
  expl_temp.f324 = coeff_14_UAV * t185;
  expl_temp.f323 = coeff_14_UAV * t184;
  expl_temp.f322 = t38;
  expl_temp.f321 = coeff_14_UAV * t183;
  expl_temp.f320 = coeff_14_UAV * t182;
  expl_temp.f319 = t437_re_tmp * 10.0;
  expl_temp.f318 = t435_re_tmp * 10.0;
  expl_temp.f317 = rt_powd_snf(t_2_array, 5.0);
  expl_temp.f316 = t436_re_tmp * 8.0;
  expl_temp.f315 = t432_re_tmp * 10.0;
  expl_temp.f314 = t433_re_tmp * 8.0;
  expl_temp.f313 = t429_re_tmp * 10.0;
  expl_temp.f312 = t430_re_tmp * 8.0;
  expl_temp.f311 = t36;
  expl_temp.f310 = t426_re_tmp * 10.0;
  expl_temp.f309 = t427_re_tmp * 8.0;
  expl_temp.f308 = t423_re_tmp * 10.0;
  expl_temp.f307 = t424_re_tmp * 8.0;
  expl_temp.f306 = t420_re_tmp * 10.0;
  expl_temp.f305 = t35;
  expl_temp.f304 = coeff_1_UAV * t61 * 10.0;
  expl_temp.f303 = t421_re_tmp * 8.0;
  expl_temp.f302 = t417_re_tmp * 10.0;
  expl_temp.f301 = coeff_1_UAV * t58 * 10.0;
  expl_temp.f300 = t418_re_tmp * 8.0;
  expl_temp.f299 = coeff_2_UAV * t59 * 8.0;
  expl_temp.f298 = coeff_1_UAV * t54 * 10.0;
  expl_temp.f297 = t34;
  expl_temp.f296 = t415_re_tmp * 8.0;
  expl_temp.f295 = coeff_2_UAV * t55 * 8.0;
  expl_temp.f294 = coeff_1_UAV * t50 * 10.0;
  expl_temp.f293 = coeff_2_UAV * t51 * 8.0;
  expl_temp.f292 = coeff_2_UAV * t47 * 8.0;
  expl_temp.f291 = t33;
  expl_temp.f290 = acc_gain * t199;
  expl_temp.f289 = acc_gain * t198;
  expl_temp.f288.re = -t208_re;
  expl_temp.f288.im = -t208_im;
  expl_temp.f287 = t32;
  expl_temp.f286.re = -t207_re;
  expl_temp.f286.im = -t207_im;
  expl_temp.f285.re = -t206_re;
  expl_temp.f285.im = -t206_im;
  expl_temp.f284.re = -t205_re;
  expl_temp.f284.im = -t205_im;
  expl_temp.f283.re = -t204_re;
  expl_temp.f283.im = -t204_im;
  expl_temp.f282.re = -t203_re;
  expl_temp.f282.im = -t203_im;
  expl_temp.f281.re = -t202_re;
  expl_temp.f281.im = -t202_im;
  expl_temp.f280.re = -t201_re;
  expl_temp.f280.im = -t201_im;
  expl_temp.f279.re = -t200_re;
  expl_temp.f279.im = -t200_im;
  expl_temp.f278.re = -t181_re;
  expl_temp.f278.im = -coeff_11_UAV;
  expl_temp.f277 = speed_gain * t176;
  expl_temp.f276 = t31;
  expl_temp.f275 = speed_gain * t175;
  expl_temp.f274 = speed_gain * t174;
  expl_temp.f273 = coeff_13_UAV * t176;
  expl_temp.f272 = coeff_14_UAV * t175;
  expl_temp.f271 = coeff_15_UAV * t174;
  expl_temp.f270 = t137 * t_landing;
  expl_temp.f269 = coeff_7_UAV * t176;
  expl_temp.f268 = coeff_8_UAV * t175;
  expl_temp.f267 = coeff_9_UAV * t174;
  expl_temp.f266 = t134 * t_landing;
  expl_temp.f265 = t30;
  expl_temp.f264 = acc_gain * 2.0;
  expl_temp.f263 = coeff_1_UAV * t176;
  expl_temp.f262 = coeff_2_UAV * t175;
  expl_temp.f261 = coeff_3_UAV * t174;
  expl_temp.f260 = coeff_13_UAV * t173;
  expl_temp.f259 = coeff_14_UAV * t171;
  expl_temp.f258 = coeff_13_UAV * t172;
  expl_temp.f257 = coeff_15_UAV * t167;
  expl_temp.f256 = coeff_14_UAV * t169;
  expl_temp.f255 = coeff_13_UAV * t170;
  expl_temp.f254 = coeff_15_UAV * t164;
  expl_temp.f253 = t29;
  expl_temp.f252 = coeff_14_UAV * t166;
  expl_temp.f251 = coeff_13_UAV * t168;
  expl_temp.f250 = coeff_15_UAV * t161;
  expl_temp.f249 = coeff_14_UAV * t163;
  expl_temp.f248 = coeff_13_UAV * t165;
  expl_temp.f247 = coeff_15_UAV * t158;
  expl_temp.f246 = coeff_14_UAV * t160;
  expl_temp.f245 = coeff_13_UAV * t162;
  expl_temp.f244 = t120 * t_9_array;
  expl_temp.f243 = coeff_15_UAV * t155;
  expl_temp.f242 = t28;
  expl_temp.f241 = coeff_14_UAV * t157;
  expl_temp.f240 = coeff_13_UAV * t159;
  expl_temp.f239 = t116 * t_8_array;
  expl_temp.f238 = coeff_7_UAV * t173;
  expl_temp.f237 = coeff_15_UAV * t153;
  expl_temp.f236 = coeff_14_UAV * t154;
  expl_temp.f235 = coeff_13_UAV * t156;
  expl_temp.f234 = t113 * t_7_array;
  expl_temp.f233 = coeff_8_UAV * t171;
  expl_temp.f232 = coeff_7_UAV * t172;
  expl_temp.f231 = t27;
  expl_temp.f230 = coeff_15_UAV * t151;
  expl_temp.f229 = coeff_14_UAV * t152;
  expl_temp.f228 = coeff_9_UAV * t167;
  expl_temp.f227 = t109 * t_6_array;
  expl_temp.f226 = coeff_8_UAV * t169;
  expl_temp.f225 = coeff_7_UAV * t170;
  expl_temp.f224 = coeff_15_UAV * t150;
  expl_temp.f223 = coeff_9_UAV * t164;
  expl_temp.f222 = t105 * t_5_array;
  expl_temp.f221 = coeff_8_UAV * t166;
  expl_temp.f220 = t26;
  expl_temp.f219 = coeff_7_UAV * t168;
  expl_temp.f218 = coeff_9_UAV * t161;
  expl_temp.f217 = t102 * t_4_array;
  expl_temp.f216 = coeff_8_UAV * t163;
  expl_temp.f215 = coeff_7_UAV * t165;
  expl_temp.f214 = coeff_9_UAV * t158;
  expl_temp.f213 = t99 * t_3_array;
  expl_temp.f212 = coeff_8_UAV * t160;
  expl_temp.f211 = coeff_7_UAV * t162;
  expl_temp.f210 = t98 * t_9_array;
  expl_temp.f209 = t25;
  expl_temp.f208 = coeff_9_UAV * t155;
  expl_temp.f207 = t95 * t_2_array;
  expl_temp.f206 = coeff_8_UAV * t157;
  expl_temp.f205 = coeff_7_UAV * t159;
  expl_temp.f204 = t93 * t_8_array;
  expl_temp.f203 = coeff_1_UAV * t173;
  expl_temp.f202 = coeff_9_UAV * t153;
  expl_temp.f201 = coeff_8_UAV * t154;
  expl_temp.f200 = coeff_7_UAV * t156;
  expl_temp.f199 = t89 * t_7_array;
  expl_temp.f198 = t_9_array * 2.0;
  expl_temp.f197 = coeff_2_UAV * t171;
  expl_temp.f196 = coeff_1_UAV * t172;
  expl_temp.f195 = coeff_9_UAV * t151;
  expl_temp.f194 = coeff_8_UAV * t152;
  expl_temp.f193 = coeff_3_UAV * t167;
  expl_temp.f192 = t85 * t_6_array;
  expl_temp.f191 = coeff_2_UAV * t169;
  expl_temp.f190 = coeff_1_UAV * t170;
  expl_temp.f189 = coeff_9_UAV * t150;
  expl_temp.f188 = coeff_3_UAV * t164;
  expl_temp.f187 = t23;
  expl_temp.f186 = coeff_2_UAV * t166;
  expl_temp.f185 = coeff_1_UAV * t168;
  expl_temp.f184 = coeff_3_UAV * t161;
  expl_temp.f183 = coeff_2_UAV * t163;
  expl_temp.f182 = coeff_1_UAV * t165;
  expl_temp.f181 = coeff_3_UAV * t158;
  expl_temp.f180 = coeff_2_UAV * t160;
  expl_temp.f179 = t_8_array * 2.0;
  expl_temp.f178 = coeff_1_UAV * t162;
  expl_temp.f177 = coeff_3_UAV * t155;
  expl_temp.f176 = coeff_2_UAV * t157;
  expl_temp.f175 = coeff_1_UAV * t159;
  expl_temp.f174 = coeff_3_UAV * t153;
  expl_temp.f173 = coeff_2_UAV * t154;
  expl_temp.f172 = coeff_1_UAV * t156;
  expl_temp.f171 = coeff_3_UAV * t151;
  expl_temp.f170 = coeff_2_UAV * t152;
  expl_temp.f169 = t21;
  expl_temp.f168 = coeff_3_UAV * t150;
  expl_temp.f167.re = t208_re;
  expl_temp.f167.im = t208_im;
  expl_temp.f166.re = t207_re;
  expl_temp.f166.im = t207_im;
  expl_temp.f165.re = t206_re;
  expl_temp.f165.im = t206_im;
  expl_temp.f164.re = t205_re;
  expl_temp.f164.im = t205_im;
  expl_temp.f163.re = t204_re;
  expl_temp.f163.im = t204_im;
  expl_temp.f162.re = t203_re;
  expl_temp.f162.im = t203_im;
  expl_temp.f161.re = t202_re;
  expl_temp.f161.im = t202_im;
  expl_temp.f160.re = t201_re;
  expl_temp.f160.im = t201_im;
  expl_temp.f159.re = t200_re;
  expl_temp.f159.im = t200_im;
  expl_temp.f158 = t_7_array * 2.0;
  expl_temp.f157 = t2;
  expl_temp.f156 = t199;
  expl_temp.f155 = t198;
  expl_temp.f154 = t197;
  expl_temp.f153 = t196;
  expl_temp.f152 = t195;
  expl_temp.f151 = t194;
  expl_temp.f150 = t193;
  expl_temp.f149 = t192;
  expl_temp.f148 = t191;
  expl_temp.f147 = t190;
  expl_temp.f146 = t19;
  expl_temp.f145 = t189;
  expl_temp.f144 = t188;
  expl_temp.f143 = t187;
  expl_temp.f142 = t186;
  expl_temp.f141 = t185;
  expl_temp.f140 = t184;
  expl_temp.f139 = t183;
  expl_temp.f138 = t182;
  expl_temp.f137.re = t181_re;
  expl_temp.f137.im = coeff_11_UAV;
  expl_temp.f136 = t2 * t65;
  expl_temp.f135 = t_6_array * 2.0;
  expl_temp.f134 = t2 * t64;
  expl_temp.f133 = t2 * t63;
  expl_temp.f132 = t2 * t_landing;
  expl_temp.f131 = t176;
  expl_temp.f130 = t175;
  expl_temp.f129 = t174;
  expl_temp.f128 = t173;
  expl_temp.f127 = t172;
  expl_temp.f126 = t171;
  expl_temp.f125 = t170;
  expl_temp.f124 = t17;
  expl_temp.f123 = t169;
  expl_temp.f122 = t168;
  expl_temp.f121 = t167;
  expl_temp.f120 = t166;
  expl_temp.f119 = t165;
  expl_temp.f118 = t164;
  expl_temp.f117 = t163;
  expl_temp.f116 = t162;
  expl_temp.f115 = t161;
  expl_temp.f114 = t160;
  expl_temp.f113 = t_5_array * 2.0;
  expl_temp.f112 = t159;
  expl_temp.f111 = t158;
  expl_temp.f110 = t157;
  expl_temp.f109 = t156;
  expl_temp.f108 = t155;
  expl_temp.f107 = t154;
  expl_temp.f106 = t153;
  expl_temp.f105 = t152;
  expl_temp.f104 = t151;
  expl_temp.f103 = t150;
  expl_temp.f102 = t_4_array * 2.0;
  expl_temp.f101 = -coeff_18_UAV;
  expl_temp.f100 = -coeff_17_UAV;
  expl_temp.f99 = -t8;
  expl_temp.f98 = -Vz_max_control_UAV;
  expl_temp.f97 = -Vy_max_control_UAV;
  expl_temp.f96 = -Vx_max_control_UAV;
  expl_temp.f95 = -Az_max_control_UAV;
  expl_temp.f94 = -Ay_max_control_UAV;
  expl_temp.f93 = -Ax_max_control_UAV;
  expl_temp.f92 = speed_gain * t29;
  expl_temp.f91 = t_3_array * 2.0;
  expl_temp.f90 = coeff_15_UAV * t30;
  expl_temp.f89 = t8 * t_landing;
  expl_temp.f88 = t208_re_tmp * 4.0;
  expl_temp.f87 = t6 * t_landing;
  expl_temp.f86 = coeff_4_UAV * t_landing * 4.0;
  expl_temp.f85 = t4 * t_landing;
  expl_temp.f84 = coeff_15_UAV * t28;
  expl_temp.f83 = coeff_15_UAV * t27;
  expl_temp.f82 = t_2_array * 2.0;
  expl_temp.f81 = coeff_15_UAV * t26;
  expl_temp.f80 = t8 * t_9_array;
  expl_temp.f79 = coeff_15_UAV * t25;
  expl_temp.f78 = t8 * t_8_array;
  expl_temp.f77 = coeff_15_UAV * t23;
  expl_temp.f76 = t8 * t_7_array;
  expl_temp.f75 = coeff_15_UAV * t21;
  expl_temp.f74 = t8 * t_6_array;
  expl_temp.f73 = coeff_15_UAV * t19;
  expl_temp.f72 = rt_powd_snf(coeff_11_UAV, 3.0);
  expl_temp.f71 = t8 * t_5_array;
  expl_temp.f70 = coeff_15_UAV * t17;
  expl_temp.f69 = t207_re_tmp * 4.0;
  expl_temp.f68 = t8 * t_4_array;
  expl_temp.f67 = t206_re_tmp * 4.0;
  expl_temp.f66 = t8 * t_3_array;
  expl_temp.f65 = t6 * t_9_array;
  expl_temp.f64 = t205_re_tmp * 4.0;
  expl_temp.f63 = t11;
  expl_temp.f62 = t8 * t_2_array;
  expl_temp.f61 = t6 * t_8_array;
  expl_temp.f60 = t204_re_tmp * 4.0;
  expl_temp.f59 = t6 * t_7_array;
  expl_temp.f58 = t203_re_tmp * 4.0;
  expl_temp.f57 = t6 * t_6_array;
  expl_temp.f56 = t202_re_tmp * 4.0;
  expl_temp.f55 = rt_powd_snf(coeff_5_UAV, 3.0);
  expl_temp.f54 = speed_gain;
  expl_temp.f53 = pos_gain;
  expl_temp.f52 = coeff_9_ship_prediction;
  expl_temp.f51 = coeff_9_UAV;
  expl_temp.f50 = coeff_8_ship_prediction;
  expl_temp.f49 = coeff_8_UAV;
  expl_temp.f48 = coeff_7_ship_prediction;
  expl_temp.f47 = coeff_7_UAV;
  expl_temp.f46 = coeff_6_ship_prediction;
  expl_temp.f45 = coeff_6_UAV;
  expl_temp.f44 = coeff_5_ship_prediction;
  expl_temp.f43 = coeff_5_UAV;
  expl_temp.f42 = coeff_4_ship_prediction;
  expl_temp.f41 = coeff_4_UAV;
  expl_temp.f40 = coeff_3_ship_prediction;
  expl_temp.f39 = coeff_3_UAV;
  expl_temp.f38 = coeff_2_ship_prediction;
  expl_temp.f37 = coeff_2_UAV;
  expl_temp.f36 = coeff_1_ship_prediction;
  expl_temp.f35 = coeff_1_UAV;
  expl_temp.f34 = coeff_18_ship_prediction;
  expl_temp.f33 = coeff_18_UAV;
  expl_temp.f32 = coeff_17_ship_prediction;
  expl_temp.f31 = coeff_17_UAV;
  expl_temp.f30 = coeff_16_ship_prediction;
  expl_temp.f29 = coeff_16_UAV;
  expl_temp.f28 = coeff_15_ship_prediction;
  expl_temp.f27 = coeff_15_UAV;
  expl_temp.f26 = coeff_14_ship_prediction;
  expl_temp.f25 = coeff_14_UAV;
  expl_temp.f24 = coeff_13_ship_prediction;
  expl_temp.f23 = coeff_13_UAV;
  expl_temp.f22 = coeff_12_ship_prediction;
  expl_temp.f21 = coeff_12_UAV;
  expl_temp.f20 = coeff_11_ship_prediction;
  expl_temp.f19 = coeff_11_UAV;
  expl_temp.f18 = coeff_10_ship_prediction;
  expl_temp.f17 = coeff_10_UAV;
  expl_temp.f16 = acc_gain;
  expl_temp.f15 = Z_0_UAV;
  expl_temp.f14 = Y_0_UAV;
  expl_temp.f13 = X_0_UAV;
  expl_temp.f12 = Vz_min_control_UAV;
  expl_temp.f11 = Vz_0_UAV;
  expl_temp.f10 = Vy_min_control_UAV;
  expl_temp.f9 = Vy_0_UAV;
  expl_temp.f8 = Vx_min_control_UAV;
  expl_temp.f7 = Vx_0_UAV;
  expl_temp.f6 = Az_min_control_UAV;
  expl_temp.f5 = Az_0_UAV;
  expl_temp.f4 = Ay_min_control_UAV;
  expl_temp.f3 = Ay_0_UAV;
  expl_temp.f2 = Ax_min_control_UAV;
  expl_temp.f1 = Ax_0_UAV;
  ft_1(&expl_temp, c_data, c_size, ceq_data, ceq_size, c_gradient_data,
       c_gradient_size, ceq_gradient_data, ceq_gradient_size);
}

/*
 * File trailer for compute_constraints_and_constraints_gradient_w_ship_coeff.c
 *
 * [EOF]
 */
