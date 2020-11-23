function show_pano_in_3D( pano_img )
%SHOW_PANO_IN_3D - show panorama points on a sfere in 3D
    th = linspace(-pi/2,pi/2,size(pano_img,1));
    fi = linspace(-pi,pi,size(pano_img,2));
    [X,Y] = meshgrid(th, fi);
    pts = [cos(X(:)).*cos(Y(:)) cos(X(:)).*sin(Y(:)) -sin(X(:))]';
    R = pano_img(:,:,1)';
    G = pano_img(:,:,2)';
    B = pano_img(:,:,3)';
    rgb = [R(:), G(:), B(:)];
    figure; axis equal;
    pcshow(pts(:,1:100:end)', rgb(1:100:end,:));
end

