folderin = 'C:\Users\Administrator\Desktop\20171125\';
folderout = 'C:\Users\Administrator\Desktop\20171125_rename\';
files = dir([folderin '\*.jpg']);
for i = 1:1:numel(files)
    oldname = files(i).name;
    I = imread([folderin,oldname]);
    %[pathstr, name, ext] = fileparts(oldname) ;
        newname = strcat(buling(i),'.jpg');
        imwrite(I,[folderout,newname],'jpg');
end