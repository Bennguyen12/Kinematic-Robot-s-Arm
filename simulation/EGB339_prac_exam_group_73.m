%% Solution for EGB339 prac for 2020

function [init_xy, dest_xy] = EGB339_prac_exam_group_73(dobot_sim, init_positions, dest_positions,use_vision)

if use_vision == true
    
    
    
    % do computer vision on initial_positions and destination_positions arguments which are images
    
    % add your code here which should include move robot arm and capture image from sim %%
    
    
    % angle of each joint
    target = [0 0 -45 45 0];
    
    dobot_sim.setJointPositions(target);
    
    % What if the robot hasn't reached the target before the next command?
    dobot_sim.getJointPositions();
    pause(20);
    image = dobot_sim.getImage();
    imshow(image);
    % you will need to calculate the xy positions and return them at the end of this section of code
    % shapes_xy needs to be
    % init_shape_xy and dest_xy need to be of size 3 x 2 and an example is shown below
    
    img_init=imrotate(init_positions,-90,'bilinear');
    
    img_dest=imrotate(dest_positions,-90,'bilinear');
    image= imrotate(image,-90,'bilinear');  %% ?
    
    [image_red_green_init,colour_init]= determine_colour_testsheet(img_init);
    inf_testsheet_init= identify_shape(image_red_green_init, colour_init);
    
    %Rearrange order  determine_colour_testsheet
    [image_red_green_dest,colour_dest]= determine_colour_testsheet(img_dest);
    inf_testsheet_dest= identify_shape(image_red_green_dest, colour_dest);
    
    [inf_worksheet,image_blue,colour_ws]= identify_shape_worksheet(image);
    
    location_worksheet_init = transfer_testsheet_worksheet(inf_worksheet, inf_testsheet_init);
    location_worksheet_dest = transfer_testsheet_worksheet(inf_worksheet, inf_testsheet_dest);
    H= find_homography(image, image_blue);
    
    coordinate_init_real= find_real_coor(H,inf_worksheet,location_worksheet_init,image);
    coordinate_dest_real= find_real_coor(H,inf_worksheet,location_worksheet_dest,image);
    
    
    init_xy=coordinate_init_real.';
    dest_xy=coordinate_dest_real.';
    
    
    
else
    
    %% Execute Pick and Place
    
    % This section iterates through each start and end coordinates
    % to pick up each object and place at the desired location
    
    
    % First movement
    pick_up(dobot_sim, init_positions(1,1), init_positions(1,2));
    drop_in(dobot_sim, dest_positions(1,1), dest_positions(1,2));
    
    % Second movement
    pick_up(dobot_sim, init_positions(2,1), init_positions(2,2));
    drop_in(dobot_sim, dest_positions(2,1), dest_positions(2,2));
    
    % Third movement
    pick_up(dobot_sim, init_positions(3,1), init_positions(3,2));
    drop_in(dobot_sim, dest_positions(3,1), dest_positions(3,2));
    
end
% Vison
    function [image_red_gr,colour]= determine_colour_testsheet(img)
        
        imred=img(:,:,1);
        imgreen=img(:,:,2);
        imblue=img(:,:,3);
        
        imr=double(imred)/255;
        imgr=double(imgreen)/255;
        imb=double(imblue)/255;
        
        imr=imr./(imr+imgr+imb);
        imgr=imgr./(imr+imgr+imb);
        
        imrthing=imr>0.6;
        imgrthing=imgr>0.6;
        
        imrthing = imopen(imrthing, strel('disk', 5));
        imgrthing = imopen(imgrthing, strel('disk', 5));
        
        image_green=regionprops(imgrthing,'Centroid','Area','Circularity');
        image_red=regionprops(imrthing,'Centroid','Area','Circularity');
        
        
        
        % colour 1- red , 0 - green
        %  colour_red= ones(1,length(image_red));
        %  colour_green=zeros(1,length(image_green));
        %  colour=[colour_red colour_green].';
        image_red_gr_combine=[image_red ;image_green];
        % Rearrange order
        centroid=cat(1,image_red_gr_combine.Centroid);
        
        local_min= find(centroid(:,2)==min(centroid(:,2)));
        
        local_max= find(centroid(:,2)==max(centroid(:,2)));
        for i=1:3
            if centroid(i,2)>centroid(local_min,2)&& centroid(i,2)<centroid(local_max,2)
                local_middle=i;
            end
        end
        image_red_gr=[image_red_gr_combine(local_min,:); image_red_gr_combine(local_middle,:);image_red_gr_combine(local_max,:)];
        colour=zeros(1,3);
        for j=1:3
            for i=1:length(image_red)
                if image_red_gr(j).Area==image_red(i).Area
                    colour(j)=1;  %red
                    
                end
                
            end
        end
    end

    function inf_testsheet= identify_shape(image_red_gr, colour)
        
        
        inf_testsheet = zeros(length(image_red_gr), 4);
        % Each Column of inf_testsheet
        %1 - Find shape 2- Circle ; 1-Square ; 0-triangle
        %2 - Area
        %3 - Size 1-Large 0-Small
        %4 - Colour
        %5 - Centroids
        
        %Store Area , colour and centroids
        Total_area=0;
        for i=1:length(image_red_gr)
            inf_testsheet(i,2) = image_red_gr(i).Area;
            inf_testsheet(i,4) = colour(i);
            %     inf_testsheet(i,5) = image_red_gr(i).Centroid;
            Total_area = Total_area + image_red_gr(i).Area;
        end
        
        avarage_total_area=Total_area/length(image_red_gr);
        % %Initialise area of each shape
        % total_area_cir=0;
        % total_area_squ=0;
        % total_area_tri=0;
        % %Initialise number each shape
        count_cir=0;
        count_squ=0;
        count_tri=0;
        Area_shape=inf_testsheet(:,2);
        for k= 1: length(image_red_gr)
            
            if image_red_gr(k).Circularity >= 0.88 && image_red_gr(k).Circularity < 1.8
                inf_testsheet(k,1)= 2;  % Circle
                %         total_area_cir = total_area_cir + inf_testsheet(k,2);
                count_cir = count_cir + 1;
                
            elseif image_red_gr(k).Circularity > 0.73 && image_red_gr(k).Circularity <0.87
                inf_testsheet(k,1)=1;     % Square
                %         total_area_squ = total_area_squ + inf_testsheet(k,2);
                count_squ = count_squ + 1;
                
            elseif image_red_gr(k).Circularity>0.5 && image_red_gr(k).Circularity <=0.73
                inf_testsheet(k,1)=0;    % Triangle
                %         total_area_tri = total_area_tri + inf_testsheet(k,2);
                count_tri = count_tri + 1;
                
            end
        end
        
        if count_tri >1
            indx=find(inf_testsheet(:,1)==0);
            
            if Area_shape(indx(1))>Area_shape(indx(2))
                inf_testsheet(indx(1),3) = 1;
                Area_shape(indx(1))=0;
                Area_shape(indx(2))=0;
            else
                inf_testsheet(indx(2),3) = 1;
                Area_shape(indx(1))=0;
                Area_shape(indx(2))=0;
            end
        end
        
        if count_squ >1
            indx=find(inf_testsheet(:,1)==1);
            
            if Area_shape(indx(1))>Area_shape(indx(2))
                inf_testsheet(indx(1),3) = 1;
                Area_shape(indx(1))=0;
                Area_shape(indx(2))=0;
            else
                inf_testsheet(indx(2),3) = 1;
                Area_shape(indx(1))=0;
                Area_shape(indx(2))=0;
            end
        end
        
        if count_cir>1
            indx=find(inf_testsheet(:,1)==2);
            
            if Area_shape(indx(1))>Area_shape(indx(2))
                inf_testsheet(indx(1),3) = 1;
                Area_shape(indx(1))=0;
                Area_shape(indx(2))=0;
            else
                inf_testsheet(indx(2),3) = 1;
                Area_shape(indx(1))=0;
                Area_shape(indx(2))=0;
            end
        end
        
        % Modify the average area to compare !!!
        for j=1:length(image_red_gr)
            if Area_shape(j) >= (0.9*avarage_total_area)
                inf_testsheet(j,3) = 1;  % Large shape
            end
        end
        
    end

    function [inf_worksheet,image_blue,colour_ws]= identify_shape_worksheet(img)
        imred=img(:,:,1);
        imgreen=img(:,:,2);
        imblue=img(:,:,3);
        
        imr=double(imred)/255;
        imgr=double(imgreen)/255;
        imb=double(imblue)/255;
        
        imr=imr./(imr+imgr+imb);
        imgr=imgr./(imr+imgr+imb);
        imb=imb./(imr+imgr+imb);
        
        imbthing=imb>0.6;
        imrthing=imr>0.6;
        imgrthing=imgr>0.6;
        
        %Filter
        imbthing = bwareaopen(imbthing, 200);
        imrthing = bwareaopen(imrthing, 200);
        imgrthing = bwareaopen(imgrthing, 200);
        
        image_green=regionprops(imgrthing,'Centroid','Area','Circularity');
        image_red=regionprops(imrthing,'Centroid','Area','Circularity');
        image_blue=regionprops(imbthing,'Centroid','Area','Circularity');
        
        % ignore rectangle
        index=find([image_red.Circularity]<0.49);
        for i=1:length(index)
            image_red(index(i),:)=[];
        end
        % colour 1- red , 0 - green
        colour_red= ones(1,length(image_red));
        colour_green=zeros(1,length(image_green));
        colour_ws=[colour_red colour_green].';
        image_red_green_ws=[image_red ;image_green];
        centroid=cat(1,image_red_green_ws.Centroid);
        
        inf_worksheet = zeros(length(image_red_green_ws), 6);
        % Each Column of inf_testsheet
        %1 - Find shape 2- Circle ; 1-Square ; 0-triangle
        %2 - Area
        %3 - Size 1-Large 0-Small
        %4 - Colour
        % Centroids   5-u ;  6-v
        
        %Store Area , colour and centroids
        Total_area=0;
        for i=1:length(image_red_green_ws)
            inf_worksheet(i,2) = image_red_green_ws(i).Area;
            inf_worksheet(i,4) = colour_ws(i);
            inf_worksheet(i,5) = centroid(i,1); % u
            inf_worksheet(i,6) = centroid(i,2);  %v
            %     inf_testsheet(i,5) = region_colour(i).Centroid;
            Total_area = Total_area + image_red_green_ws(i).Area;
        end
        
        %Initialise area of each shape
        total_area_cir=0;
        total_area_squ=0;
        total_area_tri=0;
        %Initialise number each shape
        count_cir=0;
        count_squ=0;
        count_tri=0;
        for k= 1: length(image_red_green_ws)
            
            if image_red_green_ws(k).Circularity >= 0.88 && image_red_green_ws(k).Circularity < 1.8
                inf_worksheet(k,1)= 2;  % Circle
                total_area_cir = total_area_cir + inf_worksheet(k,2);
                count_cir = count_cir + 1;
            elseif image_red_green_ws(k).Circularity >= 0.73 && image_red_green_ws(k).Circularity <0.87
                inf_worksheet(k,1)=1;     % Square
                total_area_squ = total_area_squ + inf_worksheet(k,2);
                count_squ = count_squ + 1;
                
            elseif image_red_green_ws(k).Circularity>0.5 && image_red_green_ws(k).Circularity <0.73
                inf_worksheet(k,1)=0;    % Triangle
                total_area_tri = total_area_tri + inf_worksheet(k,2);
                count_tri = count_tri + 1;
            else
                inf_worksheet(k,1)=4;
            end
            
        end
        
        average_area_circle = total_area_cir/count_cir;
        average_area_square = total_area_squ/count_squ;
        average_area_triangle = total_area_tri/count_tri;
        
        for i=1:length(image_red_green_ws)
            if inf_worksheet(i,1) == 0%If shape is a square
                if inf_worksheet(i,2) >= average_area_triangle
                    inf_worksheet(i,3) = 1;
                end
            elseif inf_worksheet(i,1) == 1%If shape is a triangle
                if inf_worksheet(i,2) >= average_area_square
                    inf_worksheet(i,3) = 1;
                end
            elseif inf_worksheet(i,1) == 2%If shape is a circle
                if inf_worksheet(i,2) >= average_area_circle
                    inf_worksheet(i,3) = 1;
                end
            end
        end
    end


    function location = transfer_testsheet_worksheet(inf_worksheet, inf_test_sheet)
        
        indx = zeros(length(inf_test_sheet(:,1)),1);
        for a=1:size(inf_worksheet)
            for b=1:size(inf_test_sheet)
                if inf_test_sheet(b,1) == inf_worksheet(a,1) && inf_test_sheet(b,3) == inf_worksheet(a,3) && inf_test_sheet(b,4) == inf_worksheet(a,4)
                    indx(b) = a;
                    break;
                end
            end
        end
        location = indx;
    end

    function H = find_homography(image_worksheet, image_blue)
        % !!! Change x-y axis
        [u_max, v_max, c] = size(image_worksheet);
        centroid=cat(1,image_blue.Centroid);
        index_left=find(centroid(:,2) < v_max/3 & centroid(:,2) > 0);
        centroid_left_u=centroid(index_left,1);
        
        centroid_left_v=centroid(index_left,2);
        
        %top left
        index_left_top=find(centroid_left_u==max(centroid_left_u));
        P(1, 1) = centroid_left_u(index_left_top);
        P(2, 1) = centroid_left_v(index_left_top);
        
        % bot left
        index_left_bot=find(centroid_left_u==min(centroid_left_u));
        P(1, 2) = centroid_left_u(index_left_bot);
        P(2, 2) = centroid_left_v(index_left_bot);
        
        index_right=find(centroid(:,2) > v_max/2+ v_max/3) ;
        
        centroid_right_u=centroid(index_right,1);
        
        centroid_right_v=centroid(index_right,2);
        
        %top right
        index_right_top=find(centroid_right_u==max(centroid_right_u));
        P(1, 3) = centroid_right_u(index_right_top);
        P(2, 3) = centroid_right_v(index_right_top);
        
        % bot right
        index_right_bot=find(centroid_right_u==min(centroid_right_u));
        P(1, 4) = centroid_right_u(index_right_bot);
        P(2, 4) = centroid_right_v(index_right_bot);
        
        
        % Points of the 4 corner blue markers
        
        Q = [345 560 ;20 560; 345 20; 20 20];
        H = simple_homography(P, Q');
        
    end

    function coordinate= find_real_coor(H,inf_worksheet,index_location,image_worksheet)
        
        [u , v]=size(inf_worksheet);
        centroid_ws=zeros(u,2);
        centroid_ws(:,1)=inf_worksheet(:,5);
        centroid_ws(:,2)=inf_worksheet(:,6);
        P=zeros(2,length(index_location));
        for i=1:length(index_location)
            P(1,i)=centroid_ws(index_location(i),1);
            P(2,i)=centroid_ws(index_location(i),2);
        end
        P=[P; ones(1,length(index_location))];
        q1=H*P(:,1);
        q2=H*P(:,2);
        q3=H*P(:,3);
        x1=q1(1)/q1(3);
        y1=q1(2)/q1(3);
        x2=q2(1)/q2(3);
        y2=q2(2)/q2(3);
        x3=q3(1)/q3(3);
        y3=q3(2)/q3(3);
        coordinate=[x1 x2 x3 ;y1 y2 y3];
        
    end






% Inverse Kinematic
    function pick_up(dobot_sim, x, y)
        
        %Move to position
        angle_1 = find_angle(x, y, 220);
        
        movement(dobot_sim, angle_1);
        pause(3);
        
        angle5 = find_angle(x, y, 200);
        movement(dobot_sim, angle5);
        pause(3);
        
        angle2 = find_angle(x, y, 190);
        movement(dobot_sim, angle2);
        pause(3);
        
        angle4 = find_angle(x, y, 180);
        movement(dobot_sim, angle4);
        pause(3);
        
        angle8 = find_angle(x, y, 160);
        movement(dobot_sim, angle8);
        pause(3);
        
        angle9 = find_angle(x, y, 150);
        movement(dobot_sim, angle9);
        pause(3);
        
        angle10 = find_angle(x, y, 150);
        movement(dobot_sim, angle10);
        pause(3);
        
        angle3 = find_angle(x, y, 56);
        movement(dobot_sim, angle3);
        pause(6);
        
        %Grab it
        dobot_sim.setSuctionCup(true);
        pause(2);
        %Stand up
        angle_last = find_angle(x, y, 130);
        movement(dobot_sim, angle_last);
        pause(2);
        angle_2_new=[angle3(1),25,-25,0,0];
        movement(dobot_sim, angle_2_new);
        pause(3);
    end

%Drop object
    function drop_in(dobot_sim, x, y)
        % Move to dest position
        angle_1 = find_angle(x, y, 180);
        
        movement(dobot_sim, angle_1);
        pause(3);
        
        %place object
        angle4 = find_angle(x, y, 200);
        movement(dobot_sim, angle4);
        pause(3);
        
        angle5 = find_angle(x, y,190);
        movement(dobot_sim, angle5);
        pause(3);
        
        angle2 = find_angle(x, y, 180);
        movement(dobot_sim, angle2);
        pause(3);
        angle6 = find_angle(x, y, 160);
        movement(dobot_sim, angle6);
        pause(3);
        
        angle7 = find_angle(x, y, 150);
        movement(dobot_sim, angle7);
        pause(3);
        
        angle8 = find_angle(x, y, 145);
        movement(dobot_sim, angle8);
        pause(3);
        %place object
        angle3 = find_angle(x, y, 63);
        movement(dobot_sim, angle3);
        %Drop
        pause(6);
        dobot_sim.setSuctionCup(false);
        
        % move backward
        pause(3);
        angle8 = find_angle(x, y, 100);
        movement(dobot_sim, angle8);
        pause(3);
        
    end

    function angle=find_angle(x,y,z)
        %x=
        %y=
        %z=
        %Arm link length
        L1=138;
        L2=135;
        L3=147;
        L4=60;
        L5=75;
        
        % Base offset
        x = x-80;
        y = y-290;
        
        theta1 = atan2(y,x);
        
        d_point=sqrt(x^2+y^2);
        h=z+L5;
        
        beta_a=atan2(h,d_point-L4);
        beta_b=(pi/2) - beta_a;
        d1=sqrt((d_point-L4)^2+h^2);
        
        d2=sqrt(d1^2+L1^2-2*d1*L1*cos(beta_b));
        
        cos_beta_c=(d2^2+L1^2-d1^2)/(2*d2*L1);
        beta_c=atan2(sqrt(1-cos_beta_c^2),cos_beta_c);
        
        cos_beta_d=(d2^2+L2^2-L3^2)/(2*d2*L2);
        beta_d=atan2(sqrt(1-cos_beta_d^2),cos_beta_d);
        
        theta2=pi-(beta_c+beta_d);
        
        cos_beta_e=(L3^2+L2^2-d2^2)/(2*L3*L2);
        beta_e=atan2(sqrt(1-cos_beta_e^2),cos_beta_e);
        
        theta3=beta_e-pi/2;
        theta3=-theta3;
        
        theta4=-theta2-theta3;
        
        theta1=rad2deg(theta1);
        theta2=rad2deg(theta2);
        theta3=rad2deg(theta3);
        theta4=rad2deg(theta4);
        angle=[theta1,theta2,theta3,theta4,0];
        
    end
    function movement(dobot_sim, angle)
        
        dobot_sim.setJointPositions(angle);
        dobot_sim.getJointPositions();
    end
end


