function[map] = buildmap(filename, M, N)

map = 255 * ones(M,N);

close all;

im = image(map);
colormap(gray(256));

im.ButtonDownFcn = @ButtonDownFcn;
im.DeleteFcn = @(hObject, eventdata)DeleteFcn(hObject, eventdata, filename);

end

%%
function ButtonDownFcn(hObject, eventdata)

persistent x0 y0 x1 y1 ;

figHandle = ancestor(hObject, 'figure');
clickType = get(figHandle, 'SelectionType');
disp(clickType);
if strcmp(clickType, 'extend')
  disp('shift+click action goes here!');
  
  if ~isempty(x0)
    hObject.CData(x0:x1, y0:y1) = 255;
  end
  
  return;
end

[xsize ysize] = size(hObject.CData);

rect = getrect;
rect(3) = rect(1) + rect(3);
rect(4) = rect(2) + rect(4);
rect = round(rect);
rect(rect < 1) = 1;

x0 = rect(2);
y0 = rect(1);
x1 = rect(4);
if x1 > xsize
    x1 = xsize;
end

y1 = rect(3);
if y1 > ysize
    y1 = ysize;
end

if strcmp(clickType, 'alt')
  
  disp('right click action goes here!');
  
  
  xc = (x0+x1)/2
  yc = (y0+y1)/2
  
  xr = (x1-x0)/2
  yr = (y1-y0)/2
  
  rmax = max(xr,yr);
  for i=-xr:1:xr
    for j=-yr:1:yr
      r = sqrt(i*i+j*j);
      if (r < rmax)
        hObject.CData(round(xc+i), round(yc+j)) = 255 - 255*(rmax-r)/rmax;
      end
    end
  end
  
else
  hObject.CData(x0:x1, y0:y1) = 0;
  
end



end

%% 
function DeleteFcn(hObject, eventdata, filename)

disp(hObject);
disp(eventdata);

map = 255 - hObject.CData;

dlmwrite(filename, map, 'delimiter', ' ');

end
