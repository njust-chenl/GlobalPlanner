function path = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;%目标点
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;%障碍物
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=zeros(10,2);%预先给变量赋值空间

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xStart;
    yNode=yStart;
    OPEN_COUNT=1;
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=0;
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);%表示OPEN的行
    OPEN(OPEN_COUNT,1)=1;
    CLOSED_COUNT=CLOSED_COUNT+1;%然后将起点放入CLOSEDlist中。
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;

%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while(1) %you have to dicide the Conditions for while loop exit 
                % if the queue(OPEN) is empty, return False; break; 
        if sum(OPEN(:,1)) == 0
            break;
        end
        i_min = min_fn(OPEN,length(OPEN(:,1)),xTarget,yTarget); 
        xNode_expanded = OPEN(i_min,2);
        yNode_expanded = OPEN(i_min,3);
        gNode_expanded = OPEN(i_min,7);
        OPEN(i_min,1) = 0;%最小的就会等于0
        CLOSED_COUNT = CLOSED_COUNT + 1;
        CLOSED(CLOSED_COUNT,1)=xNode_expanded;
        CLOSED(CLOSED_COUNT,2)=yNode_expanded;
        if xNode_expanded == xTarget && yNode_expanded == yTarget
            NoPath = 0;
            break;
        end
       exp_array=expand_array(xNode_expanded,yNode_expanded,gNode_expanded,xTarget,yTarget,CLOSED,MAX_X,MAX_Y);
        if ~isempty(exp_array)
            for m = 1:1:length(exp_array(:,1)) % For all unexpanded neighbors “m” of node “n”
                xNode = exp_array(m,1);
                yNode = exp_array(m,2);           %m=就是扩展的节点。
                % If node m is not in OPEN, push node m into OPEN 
                if  isempty(node_index(OPEN,xNode,yNode))  %m是否在OPENlist里面。
                    OPEN_COUNT=OPEN_COUNT + 1;
                    hn=distance(xNode,yNode,xTarget,yTarget)*(1/10);
                    gn=gNode_expanded + distance(xNode,yNode,xNode_expanded,yNode_expanded);
                    fn=hn + gn; 
                    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode_expanded,yNode_expanded,hn,gn,fn); %以最小的f()为扩展节点。               
                    OPEN(OPEN_COUNT,1)=1;  
                else
                    %If m Node is in OPEN and g(m) > g(n) + Cnm, which means a better path to this node is found
                    %then update parents and g , f value  
                    n_inx = node_index(OPEN, xNode,yNode);
                    if OPEN(n_inx,7) > (gNode_expanded + distance(xNode,yNode,xNode_expanded,yNode_expanded))
                        OPEN(n_inx,4) = xNode_expanded;  % Parent X val
                        OPEN(n_inx,5) = yNode_expanded;  % Parent Y val
                        OPEN(n_inx,7) = gNode_expanded + distance(xNode,yNode,xNode_expanded,yNode_expanded);  % g(m)
                        OPEN(n_inx,8) = OPEN(n_inx,6) + OPEN(n_inx,7);  % f(m)
                    end 

                end
                                    
            end
        end  
     %
     %finish the while loop
     %
     
    end %End of While Loop
    
    %Once algorithm has run The optimal path is generated by starting of at the
    %last node(if it is the target node) and then identifying its parent node
    %until it reaches the start node.This is the optimal path
    
    %
    %How to get the optimal path after A_star search?
    %please finish it
    %
   %路径的处理 
   path = [];
    if NoPath
       return;
   else
       % find the path from goal to start
       xval = xTarget;
       yval = yTarget;
       n = 1;
       index = node_index(OPEN,xval,yval);
       path(n,:) = [xval,yval];
       while index > 1
           assert(OPEN(index,1)==0, 'Something wrong!')    %当OPENlist 的第一列 index对应的坐标值为0时表示该点时路径。     
           xval = OPEN(index,4);
           yval = OPEN(index,5);%父亲坐标
           index = node_index(OPEN,xval,yval);%找父亲坐标的索引
           n = n + 1;
           path(n,:) = [xval,yval];%将找的坐标放进path中。
       end
       assert(index==1, 'Something wrong!')%当条件错误时引发错误 
       xval = OPEN(index,4);
       yval = OPEN(index,5);
       path(n+1,:) = [xval,yval];
       path = flip(path);%翻转元素顺序
   end
end
