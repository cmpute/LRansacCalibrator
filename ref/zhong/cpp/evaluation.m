summary = [];
image_count = zeros(11, 22);
fusion_count = zeros(11, 22);
for frame = 0:251
    path = sprintf('D:/±à³Ì/Lidar/data_annotated/evaluation/%06d.mat', frame);
    load(path)
    num = size(detail,1);
    image_correct = sum(detail(:,4));
    fusion_correct = sum(detail(:,5));
    summary = [summary; num, image_correct, fusion_correct,...
                        image_correct/(num * 2 - image_correct),...
                        fusion_correct/(num * 2 - fusion_correct)];
    for i = 1:num
        gt_label = detail(i,1)+1;
        img_label = detail(i,2)+1;
        fus_label = detail(i,3)+1;
        image_count(gt_label,img_label) = image_count(gt_label,img_label) + 1;
        fusion_count(gt_label,fus_label) = fusion_count(gt_label,fus_label) + 1;
    end
end

% delete unused label
image_count(3,:) = []; % sky
fusion_count(3,:) = [];
image_count(1,:) = []; % backgroud
fusion_count(1,:) = [];
image_count(:,21:22) = []; % addtional label
fusion_count(:,21:22) = [];

%custom colormap
cmap = [];
for i = 256:-1:1
    cmap = [cmap; 65536,i^2,i^2];
end
cmap = cmap / 65536;

relationship = [
    [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    [1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0];
    [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0];
    [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    [0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0];
    [0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0];
    [0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0];
    [0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1];
    [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
];

%drawing
lowest_ratio = 0.01;

figure(1)
darker_ratio = 1.2;
div = repmat(sum(image_count,2),1,20);
image_count = image_count./div;
image(100 * image_count * darker_ratio)

set(gca,'TickLength',[0,0])
set(gca,'YTick',1:9)
set(gca,'YTickLabel',{'building','road','vegetation','sidewalk','car','pedestrian','cyclist','signage','fence'})
set(gca,'XAxisLocation','top')
set(gca,'XTick',1:20)
set(gca,'XTickLabel',{'road','sidewalk','building','wall','fence','pole','traffic light','traffic sign','vegetation','terrain','sky','person','rider','car','truck','bus','train','motorcycle','bicycle','traffic facility'});
set(gca,'XTickLabelRotation', 70)
colormap(cmap);

for i= 1:9
    for j = 1:20
        if(image_count(i,j)<lowest_ratio)
            continue
        end
        txt = sprintf('%.3f',image_count(i,j));
        if(relationship(i,j)==1)
            text(j,i,txt(2:end), 'HorizontalAlignment', 'center', 'FontWeight', 'bold')
        else
            text(j,i,txt(2:end), 'HorizontalAlignment', 'center')
        end
    end
end


figure(2)
darker_ratio = 1.2;
div = repmat(sum(fusion_count,2),1,20);
fusion_count = fusion_count./div;
image(100 * fusion_count * darker_ratio)

set(gca,'TickLength',[0,0])
set(gca,'YTick',1:9)
set(gca,'YTickLabel',{'building','road','vegetation','sidewalk','car','pedestrian','cyclist','signage','fence'})
set(gca,'XAxisLocation','top')
set(gca,'XTick',1:20)
set(gca,'XTickLabel',{'road','sidewalk','building','wall','fence','pole','traffic light','traffic sign','vegetation','terrain','sky','person','rider','car','truck','bus','train','motorcycle','bicycle','traffic facility'});
set(gca,'XTickLabelRotation', 70)
colormap(cmap);

for i= 1:9
    for j = 1:20
        if(fusion_count(i,j)<lowest_ratio)
            continue
        end
        txt = sprintf('%.3f',fusion_count(i,j));
        if(relationship(i,j)==1)
            text(j,i,txt(2:end), 'HorizontalAlignment', 'center', 'FontWeight', 'bold')
        else
            text(j,i,txt(2:end), 'HorizontalAlignment', 'center')
        end
    end
end