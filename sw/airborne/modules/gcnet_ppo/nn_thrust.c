#include "nn_thrust.h"
#include <stdio.h>
#include <math.h>

const float nn_thrust_weights_fc1[] = {
0.6684572696685791, 0.27626726031303406, -0.14709796011447906, -0.5048384666442871, 0.17589302361011505, -0.28608354926109314, 0.4949333071708679,
0.9418817758560181, 0.4132333993911743, 0.5208235383033752, 0.16144247353076935, -0.45496150851249695, 0.024033287540078163, 0.06259766966104507,
0.10793554782867432, 0.6558358669281006, 0.11073829978704453, -0.38869163393974304, 0.3791394829750061, -0.20277434587478638, -0.28032562136650085,
-0.46167322993278503, 0.04075358435511589, 0.17709606885910034, 0.1573416292667389, -0.12142767012119293, -0.496632844209671, -0.2726876139640808,
-0.07135407626628876, -0.016563614830374718, -0.01688566245138645, -0.28953397274017334, 0.1468208283185959, -0.6559873223304749, -0.2871839106082916,
0.8293523192405701, -0.2871038019657135, -0.3097744584083557, 0.12625469267368317, -0.4321254789829254, -0.3507717251777649, 0.2903285026550293,
-0.5612381100654602, 0.6603149175643921, 0.240953728556633, -0.5801730155944824, 0.29933780431747437, -0.1848141998052597, 0.554324209690094,
-0.39018386602401733, -0.37196677923202515, 0.9095621109008789, 0.7784013748168945, 0.6123208403587341, -0.3014664053916931, -0.16471056640148163,
0.1782715767621994, 0.6320366263389587, 0.5229137539863586, -0.37350645661354065, -0.7079101204872131, 0.04579506441950798, -0.33767759799957275,
-0.48877212405204773, 0.3433208465576172, 0.3491900861263275, -0.5351037383079529, -0.3355118930339813, -0.46650806069374084, -0.1972522884607315,
-0.1003262996673584, 0.3641970157623291, 0.04646986350417137, -0.4285582900047302, -0.4086405634880066, -0.14982905983924866, 0.4721556305885315,
0.06045624986290932, 0.20997688174247742, 0.669231116771698, -0.02814672514796257, 0.3565196096897125, -0.12753280997276306, -0.40598514676094055,
0.4751240611076355, 0.8686538934707642, 0.1283084899187088, -0.7312808036804199, 0.46314674615859985, 0.07043259590864182, 0.19847728312015533,
0.6069784760475159, 0.37587836384773254, -0.47656652331352234, -0.1129620224237442, -0.7799530029296875, -0.37303033471107483, 0.09674517810344696,
0.08242758363485336, -0.3833827078342438, -0.21938535571098328, 0.10332819819450378, -0.03200814127922058, -0.09848315268754959, 0.8424599170684814,
-0.3092230260372162, -0.18199372291564941, 0.5300654172897339, 0.3336145281791687, 0.49125176668167114, -0.00030068334308452904, 0.47620731592178345,
-0.8179184794425964, 0.9482081532478333, 0.6006855368614197, -0.82494056224823, -0.03286394476890564, 0.0868510901927948, -0.11126749217510223,
0.7444549202919006, 0.608025074005127, -1.0044673681259155, -0.23379157483577728, 0.17938748002052307, -0.24545902013778687, -0.539896547794342,
0.29709842801094055, 0.8661707043647766, 0.3178166449069977, 0.6454864144325256, -0.45524609088897705, 0.05895249918103218, 0.31494513154029846,
0.4087887406349182, -0.1703185737133026, 0.20358356833457947, 0.6341149806976318, 0.5337395668029785, 0.2775332033634186, 0.13127128779888153,
-0.32451191544532776, 0.04532440006732941, 0.2675628662109375, -0.07699621468782425, 0.6282538175582886, -0.38489535450935364, -0.527666449546814,
-0.7359553575515747, -0.8494068384170532, 0.36154088377952576, 0.15768365561962128, -0.018768006935715675, -0.0001118040963774547, -0.6760014295578003,
0.5042939186096191, 0.5240745544433594, -0.14496518671512604, 0.07310060411691666, -0.0439319871366024, 0.3717767298221588, 0.06920938193798065,
-0.7149724364280701, 0.13867752254009247, 0.4070477783679962, 1.4815471172332764, 0.4878008961677551, -0.25373736023902893, 0.24397313594818115,
-0.16258685290813446, -0.12539023160934448, 1.019998550415039, 0.3076724410057068, 0.506469190120697, 0.09048239141702652, 0.31910407543182373,
-1.2173826694488525, -1.1775821447372437, 0.8550457954406738, -0.40024590492248535, 0.47688183188438416, 0.04562951251864433, -0.3656804859638214,
0.3161635398864746, -0.698082447052002, -0.25789400935173035, -0.0708642303943634, 0.10166658461093903, 0.31479474902153015, 0.4994184970855713,
-0.6226828694343567, -0.6015439033508301, 0.7900235056877136, 0.24882546067237854, -0.13402599096298218, -0.15655450522899628, 0.32901132106781006,
-1.3085271120071411, -0.05015243589878082, 0.1776803880929947, -0.140252947807312, 0.22420617938041687, 0.08848746865987778, -0.2615821957588196,
-0.11565940827131271, 0.706531286239624, 0.24136057496070862, 0.012605423107743263, -0.06932110339403152, -0.4848876893520355, 0.2860115170478821,
-0.9600878953933716, -0.27629536390304565, 0.643826425075531, 0.4985606074333191, 0.99017733335495, -0.24694348871707916, 0.07175076007843018,
-0.1122012808918953, 0.2446776032447815, -0.026829734444618225, -0.05468827486038208, -0.5217438340187073, 0.14177703857421875, 0.10587023198604584
};

const float nn_thrust_biases_fc1[] = {
-0.46307462453842163, 1.2554233074188232, 1.2640434503555298, 0.03330643102526665, 0.3019617199897766, -0.08441021293401718, -1.7619761228561401, 0.4874589145183563, 0.9948171973228455, 0.332977831363678, -1.6788420677185059, 1.3296936750411987, 1.09756600856781, -0.5572874546051025, -1.7669477462768555, 0.32091984152793884, 0.19984471797943115, 0.3746360242366791, 0.5736422538757324, 0.2910445034503937, 0.03685561195015907, -0.15165701508522034, -0.49597373604774475, 0.6082852482795715, 0.6918460726737976, -0.15588347613811493, -1.526419997215271, 0.08222442120313644, 0.3375769853591919, -0.05046512559056282, -0.044222284108400345, 0.43540430068969727
};

const float nn_thrust_weights_fc2[] = {
0.3934491276741028, 0.5358460545539856, -0.7000531554222107, 0.12267803400754929, 0.29063722491264343, 0.4114469289779663, -1.3079099655151367, -0.49752286076545715, -0.48194190859794617, 0.2374061495065689, -1.2240819931030273, -0.6308515071868896, 0.4287484884262085, -0.6914631724357605, -1.389930009841919, 0.275301069021225, 0.5253419876098633, 0.6417025923728943, 0.24606835842132568, 0.24175666272640228, 0.37940654158592224, 0.4935588836669922, 0.11146030575037003, 0.6116238236427307, 0.35136136412620544, 0.4331153929233551, -1.6220442056655884, 0.3623497188091278, 0.18291659653186798, 0.2860714793205261, -0.579090416431427, 0.3583308458328247
};

const float nn_thrust_biases_fc2[] = {
-0.27381327748298645
};

void nn_thrust_linear(const float* weights, const float* biases, const float* input, int in_features, int out_features, float* output) {
    for (int i = 0; i < out_features; ++i) {
        float neuron = biases[i];
        for (int j = 0; j < in_features; ++j) {
            neuron += input[j] * weights[i * in_features + j];
        }
        output[i] = neuron;
    }
}

void nn_thrust_relu(float* input, int size) {
    for (int i = 0; i < size; ++i) {
        input[i] = fmaxf(0, input[i]);
    }
}

void nn_thrust_tanh(float* input, int size) {
    for (int i = 0; i < size; ++i) {
        input[i] = tanh(input[i]);
    }
}

void nn_thrust_forward(const float* input, float* output) {
    float fc1_output[32];
    nn_thrust_linear(nn_thrust_weights_fc1, nn_thrust_biases_fc1, input, 7, 32, fc1_output);
    nn_thrust_relu(fc1_output, 32);
    nn_thrust_linear(nn_thrust_weights_fc2, nn_thrust_biases_fc2, fc1_output, 32, 1, output);
}
