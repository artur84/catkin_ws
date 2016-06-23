a = load('log.txt');

[ln,cl]=size(a);
figure, hold;
axis("equal");

begin_plot = 1;
end_plot = 1;
color = 1;

for i=1:ln
  if (a(begin_plot,1) ~= a(i,1)  | i == ln)
    
    end_plot = i-1;
    if(i==ln)
      end_plot = i;
    endif

    clr = int2str(color);
    clr = strcat(clr,"-*");       
    h = plot(a(begin_plot:end_plot,2),a(begin_plot:end_plot,3),clr);
    xlabel("x(meters)");
    ylabel("y(meters)");

    set (h, "linewidth", 4) 
    
    begin_plot = i;

    %change color
    if(color >= 5)
      color = 1;
    else
      color = color + 1;
    endif

  endif
end

axis("tight","equal")
     