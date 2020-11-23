% clear; close all; 

addpath(fullfile(pwd,'codes'));

%% settings
debug = true;

dataset_dir = fullfile(pwd,'..','SIE factory datasets','proc','2019-09-28_16.11.53');
pts_file = fullfile(dataset_dir,'2019-09-28_16_txt.ply');
pano_file = fullfile(dataset_dir,'pano','pano-poses.csv');
save_directory = fullfile(pwd,'tmp');

% params of the generated image (one exapmpe)
pano_id = 50;                   % example for image id
f = 2000;                       % focal length
u0 = 1920/2;                    % principal point u0
v0 = 1080/2;                    % principal point v0
img_size = [1920 1080];         % image size
K = [f 0 u0; 0 f v0; 0 0 1];    % the calibration matrix 
R = [1  0           0; ...
    0   cos(pi/2)     -sin(pi/2); ...
    0   sin(pi/2)     cos(pi/2)];	% some rotation


%% process
% load points in 3D & panorama poses
pts = load_pts( pts_file );
[ pano_images, pano_poses, pano_C, pano_q ] = load_pano_poses( pano_file );

% show pointcloud
% show panorama coordinate systems
if debug
    show_pointcloud( pts ); 
    subfig(3,3,1,gcf); hold on;
    show_pano_in_world( pano_C, pano_q, pano_poses, pano_images );
end 

% load panorama
pano_img = imread(fullfile(dataset_dir,'pano',pano_images{pano_id}));
figure; imshow(pano_img); subfig(3,3,2,gcf); title(sprintf('Panorama "%s"',pano_images{pano_id}));

% projection of sfere to plane  
% -> we assume projection matrix P = K R q2r(pano_q(:,pano_id)) [I pano_C(:,pano_id)];
iQ = q2r(pano_q(:,pano_id))' * R' * inv(K);
[X,Y] = meshgrid(1:img_size(1), 1:img_size(2));
X = X(:); Y = Y(:);
proj = iQ * [X, Y, ones(length(X),1)]';
proj = proj .* (ones(3,1) * 1./sqrt(sum(proj.^2)));    
th_proj = -asin(proj(3,:));
fi_proj = atan2(proj(2,:)./cos(th_proj), proj(1,:)./cos(th_proj));
uv = [((th_proj / (pi/2)) + 1) / 2 * size(pano_img,1) + 1; ...
   	  ((fi_proj / pi) + 1) / 2 * size(pano_img,2) + 1]; 

% show panorama points on a sfere in 3D
if debug
    show_pano_in_3D( pano_img ); 
    subfig(3,3,3,gcf); hold on; title('Rendered image on a sfere');
    pcshow(proj');   % show the area of rendered image
end 

% bilinear interpolation from original image
img = bilinear_interpolation( img_size, uv, pano_img );
figure(); imshow(img); %set(gca, 'XDir','reverse'); 
subfig(3,3,4,gcf); title('Rendered image');

% project the factory pointcloud by related projection matrix P
P = K * R * q2r(pano_q(:,pano_id))' * [eye(3) -pano_C(:,pano_id)];
[ fpts, frgb ] = filterFieldView( ...
    struct('R', R * q2r(pano_q(:,pano_id)), 'C', pano_C(:,pano_id)), ...
    pts(1:3,:), pts(4:6,:));
uvs = round(h2a(P * a2h(fpts))); % projected points into image plane

if debug
    % show selected pointcloud
    show_selected_pointcloud( fpts, frgb, pano_q, pano_C, pano_id  );
    subfig(3,3,5,gcf); title('Selected subset of 3D points for projection');
    
    %show projected points in 2D image
    show_projected_pts( uvs, img_size, frgb ); 
    subfig(3,3,6,gcf); title('Projected 3D points into the image using P');
end

%% Create COLMAP files
% 1) add camera
cameras = containers.Map('KeyType','int64','ValueType','any');
cameras(1) = struct('camera_id',1,'model','SIMPLE_PINHOLE','width',img_size(1),...
    'height',img_size(2),'params',[f u0 v0]);

% 2) add images (one image in this example)
images = containers.Map('KeyType','int64','ValueType','any');
img_R = R * q2r(pano_q(:,pano_id))';
images(1) = struct('image_id',1,'q',r2q(img_R)',...
    'R',img_R,'t',-img_R*pano_C(:,pano_id),'camera_id',1,'name','pokus001.jpg','xys',uvs', ...
    'point3D_ids',1:size(uvs,2));

% 3) add poitns
points3D = containers.Map('KeyType', 'int64', 'ValueType', 'any');
for i = 1:size(fpts,2)
    points3D(i) = struct('point3D_id',i,'xyz',fpts(:,i),'rgb',frgb(:,i),'error',0,...
        'track',[1 0]);
end

saveColmap( save_directory, cameras, images, points3D );

