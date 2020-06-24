% read YUV Dictionary
% clear; close all; clc;

% load Dictionary
% YUV_Dict = load('GoodDictionaryCyberZooPinkMat.dat');
YUV_Dict = load('Dictionary_00000.dat');
% YUV_Dict = load('Dictionary_big_flowers.dat');

% params
n_textons = 20; % number of visual words
patch_size = 6; % size of one patch

% extract YUV components
U_ALL = YUV_Dict(1:4:end,1);
Y1_ALL = YUV_Dict(2:4:end,1);
V_ALL = YUV_Dict(3:4:end,1);
Y2_ALL = YUV_Dict(4:4:end,1);

Dictionary = zeros(n_textons,patch_size*patch_size*3);


hh = figure();

for w = 1:n_textons
    subplot(ceil(sqrt(n_textons)), ceil(sqrt(n_textons)), w);
    
    % extract YUV per texton
    U = U_ALL((w-1)*18+1:w*18, 1);
    Y1 = Y1_ALL((w-1)*18+1:w*18, 1);
    V = V_ALL((w-1)*18+1:w*18, 1);
    Y2 = Y2_ALL((w-1)*18+1:w*18, 1);
    
    % conversion
    R1 = Y1 + 1.4022 .* (V - 128);
    G1 = Y1 - 0.3456 .* (U - 128) - (0.7145 .* (V - 128));
    B1 = Y1 + 1.7710 .* (U - 128);
    R2 = Y2 + 1.4022 .* (V - 128);
    G2 = Y2 - 0.3456 .* (U - 128) - (0.7145 .* (V - 128));
    B2 = Y2 + 1.7710 .* (U - 128);
    
    R = zeros(patch_size,patch_size);
    G = zeros(patch_size,patch_size);
    B = zeros(patch_size,patch_size);
    R(:,1:2:end) = reshape(R1,3,6)';
    R(:,2:2:end) = reshape(R2,3,6)';
    G(:,1:2:end) = reshape(G1,3,6)';
    G(:,2:2:end) = reshape(G2,3,6)';
    B(:,1:2:end) = reshape(B1,3,6)';
    B(:,2:2:end) = reshape(B2,3,6)';
    
    % clip the values into range [0, 255]
    R = max(0, min(R, 255));
    G = max(0, min(G, 255));
    B = max(0, min(B, 255));
    % R = min(max(R, 0),255);
    % G = min(max(G, 0),255);
    % B = min(max(B, 0),255);

    % form bgr image
    R_norm = R./255;
    G_norm = G./255;
    B_norm = B./255;
    rgbimg (:,:,1) = B_norm;
    rgbimg (:,:,2) = G_norm;
    rgbimg (:,:,3) = R_norm;
    
    Dictionary(w,1:patch_size*patch_size) = reshape(rgbimg(:,:,1)',1,36);
    Dictionary(w,patch_size*patch_size+1:patch_size*patch_size*2) = reshape(rgbimg(:,:,2)',1,36);
    Dictionary(w,patch_size*patch_size*2+1:patch_size*patch_size*3) = reshape(rgbimg(:,:,3)',1,36);
    
    imshow(rgbimg);
    title(num2str(w))
end

% save dictionary_visualwords2_rgb.mat Dictionary