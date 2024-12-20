%inputs
Feed_Force = 20; %N
MC_Wood = 25; %moisture percentage
Density_Wood = 550; %kg/m^3
Length = 90; %mm
Area = Length*Length; %mm^2
Pitch = 9.525; %mm
Sprocket_Drive_Teeth = 6;
Drive_Sprocket_RPM = 3000;
Drive_Sprocket_w = Drive_Sprocket_RPM*0.10472;
S_tooth_spacing = 8;
Coefficients = [109.56,-2.39,0.16,-0.62,264.83,62.77,-8.30,0.31;88.24,-2.20,0.15,-0.65,238.61,41.02,-6.68,0.30;43.03,-1.75,0.15,-0.33,131.68,98.63,-6.59,0.37];

%set values
MC_Ave = 20.57;
Density_Ave = 544.8;
Chain_Speed_Ave = 6.51; %m/s
Depth_Ave = 0.362; %mm
Depth_Overload = 0.5; %mm

%initialize x for feed speed calculation
syms x

%Calculated values
Chain_Speed = ((Pitch/1000)*Sprocket_Drive_Teeth*Drive_Sprocket_w)/pi; %fixed wrong equation

%Calculate feed speed based on feed force and other input values
Feed_Speed_Eqn =  Feed_Force == Coefficients(3,1)...
+(Coefficients(3,2)*(MC_Wood-MC_Ave))...
+(Coefficients(3,3)*(Density_Wood-Density_Ave))...
+(Coefficients(3,4)*(Chain_Speed-Chain_Speed_Ave))...
+(Coefficients(3,5)*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Ave))...
+(Coefficients(3,6)*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Overload))...
+(Coefficients(3,7)*((MC_Wood-MC_Ave)*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Ave)))...
+(Coefficients(3,8)*((Density_Wood-Density_Ave)*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Ave)));

Feed_Speed = round(solve(Feed_Speed_Eqn,x),5);
Depth_of_Cut = round((Feed_Speed/Chain_Speed)*Pitch*S_tooth_spacing,5);

%if the solve above results in the depth of cut condition where b5 should 
%be 0, then it needs to be recalculated again with that set to 0
if Depth_of_Cut-Depth_Ave > Depth_Overload-Depth_Ave 
    %round speed and depth for completion
    Feed_Speed = round(Feed_Speed,3);
    Depth_of_Cut = round(Depth_of_Cut,3);
    
    %need to calculate chain force and cutting force now using the depth of
    %cut value from above
    
    Chain_Force = round(Coefficients(1,1)...
    +(Coefficients(1,2)*(MC_Wood-MC_Ave))...
    +(Coefficients(1,3)*(Density_Wood-Density_Ave))...
    +(Coefficients(1,4)*(Chain_Speed-Chain_Speed_Ave))...
    +(Coefficients(1,5)*(Depth_of_Cut-Depth_Ave))...
    +(Coefficients(1,6)*(Depth_of_Cut-Depth_Overload))...
    +(Coefficients(1,7)*((MC_Wood-MC_Ave)*(Depth_of_Cut-Depth_Ave)))...
    +(Coefficients(1,8)*((Density_Wood-Density_Ave)*(Depth_of_Cut-Depth_Ave))),1);
    
    Cutting_Force = round(Coefficients(2,1)...
    +(Coefficients(2,2)*(MC_Wood-MC_Ave))...
    +(Coefficients(2,3)*(Density_Wood-Density_Ave))...
    +(Coefficients(2,4)*(Chain_Speed-Chain_Speed_Ave))...
    +(Coefficients(2,5)*(Depth_of_Cut-Depth_Ave))...
    +(Coefficients(2,6)*(Depth_of_Cut-Depth_Overload))...
    +(Coefficients(2,7)*((MC_Wood-MC_Ave)*(Depth_of_Cut-Depth_Ave)))...
    +(Coefficients(2,8)*((Density_Wood-Density_Ave)*(Depth_of_Cut-Depth_Ave))),1);
else %fails condition so b5 set to 0

    Feed_Speed_Eqn =  Feed_Force == Coefficients(3,1)...
    +(Coefficients(3,2)*(MC_Wood-MC_Ave))...
    +(Coefficients(3,3)*(Density_Wood-Density_Ave))...
    +(Coefficients(3,4)*(Chain_Speed-Chain_Speed_Ave))...
    +(Coefficients(3,5)*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Ave))...
    +(Coefficients(3,6)*0*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Overload))...
    +(Coefficients(3,7)*((MC_Wood-MC_Ave)*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Ave)))...
    +(Coefficients(3,8)*((Density_Wood-Density_Ave)*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Ave)));
    Feed_Speed = round(solve(Feed_Speed_Eqn,x),3);
    Depth_of_Cut = round((Feed_Speed/Chain_Speed)*Pitch*S_tooth_spacing,3);
    %need to calculate chain force and cutting force now using the depth of
    %cut value from above

    Chain_Force = round(Coefficients(1,1)...
    +(Coefficients(1,2)*(MC_Wood-MC_Ave))...
    +(Coefficients(1,3)*(Density_Wood-Density_Ave))...
    +(Coefficients(1,4)*(Chain_Speed-Chain_Speed_Ave))...
    +(Coefficients(1,5)*(Depth_of_Cut-Depth_Ave))...
    +(Coefficients(1,6)*0*(Depth_of_Cut-Depth_Overload))...
    +(Coefficients(1,7)*((MC_Wood-MC_Ave)*(Depth_of_Cut-Depth_Ave)))...
    +(Coefficients(1,8)*((Density_Wood-Density_Ave)*(Depth_of_Cut-Depth_Ave))),1);

    Cutting_Force = round(Coefficients(2,1)...
    +(Coefficients(2,2)*(MC_Wood-MC_Ave))...
    +(Coefficients(2,3)*(Density_Wood-Density_Ave))...
    +(Coefficients(2,4)*(Chain_Speed-Chain_Speed_Ave))...
    +(Coefficients(2,5)*(Depth_of_Cut-Depth_Ave))...
    +(Coefficients(2,6)*0*(Depth_of_Cut-Depth_Overload))...
    +(Coefficients(2,7)*((MC_Wood-MC_Ave)*(Depth_of_Cut-Depth_Ave)))...
    +(Coefficients(2,8)*((Density_Wood-Density_Ave)*(Depth_of_Cut-Depth_Ave))),1);
end

%convert sym values to doubles to read easier
Feed_Speed = double(Feed_Speed);
Depth_of_Cut = double(Depth_of_Cut);
Chain_Force = double(Chain_Force);
Cutting_Force = double(Cutting_Force);

%With the chain force the required motor torque can be calculated
Motor_Torque = ((Pitch/1000)*Sprocket_Drive_Teeth*Chain_Force)/pi;

%cutting efficiency can be calculated as well - increases for larger wood
%pieces, a square wood piece may be easiest to calculate
Cutting_Efficiency = ((Feed_Speed*1000)*Length)/(Chain_Force*Chain_Speed); %mm^2/j

%calculating against performance attributes
time_of_cut = (Length/1000)/Feed_Speed;
Cutting_Energy_Required = Area/Cutting_Efficiency;
