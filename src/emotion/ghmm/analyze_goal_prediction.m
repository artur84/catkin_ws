clear all

list = load('goals.txt');

[ln,cl]=size(list);
%  figure, hold;
%  axis("equal");

begin_track = 1;
end_track = 1;
normalized(1:10,1:3)=0;
error_matrix(:,:) = 0;
track_number = 1;

for i=1:ln
  if (list(begin_track,1) ~= list(i,1)  | i == ln)
    
    %the last item is the final position
    if(i==ln)
      end_track = i-1;
      goal_index = i;
    else
      end_track = i-2;
      goal_index = i-1;
    endif

    final_goal = [list(goal_index,2),list(goal_index,3)];

    current_list = list(begin_track:end_track,:);
    samples = size(current_list,1);
    
    %discard small tracks
    if (samples>2)    

      factor = samples/10;
      
      if (factor < 1)
        for k=1:10
          index = ceil(k*factor);
          normalized(k,2:3) = current_list(index,2:3);
        end
      elseif (factor > 1)
        old_index = 0;
        for k=1:10
          index = ceil(k*factor);
          normalized(k,2) = mean(current_list(old_index+1:index,2));
          normalized(k,3) = mean(current_list(old_index+1:index,3));
          old_index = index;
        end
      else
        normalized(1:10,2:3) = current_list(1:10,2:3);
      endif

      for k=1:10
        predicted_goal = normalized(k,2:3);
        error = sqrt((final_goal(1,1)-predicted_goal(1,1))^2+
                  ((final_goal(1,2)-predicted_goal(1,2))^2));

        error_matrix(k,track_number) = error;
      end
      ++track_number;

    endif
 
    begin_track = i;

  endif
end
error_matrix;

figure;
h1 = plot(error_matrix);
xlabel("trajectory percentage (x10)");
ylabel("mean error (m)");
set (h1, "linewidth", 4); 
grid on;

figure; 
mean_error = mean(error_matrix');
h2 = plot(mean_error,"*-");
xlabel("trajectory percentage (x10)");
ylabel("mean error (m)");
set (h2, "linewidth", 4); 
grid on;

