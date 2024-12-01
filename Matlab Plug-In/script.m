%comments for automatic ModelCenter import
% variable: Length double input default=101.6 units=mm
% variable: Pitch double input default=9.525 units=mm
% variable: Sprocket_Drive_Teeth double input default=6
% variable: Drive_Sprocket_RPM double input default=3000 units=RPM
% variable: S_tooth_spacing double input default=8
% variable: BatteryCapacity double input default=4 units=Ah
% variable: Feed_Force double input default=20 units=N
% variable: MC_Wood double input default=25 units=%
% variable: Bar_Length double input default=35 units=m
% variable: Grip_Length double input default=15 units=m
% variable: MotorTorque double input default=1.2 units=N-m
% variable: Density_Wood double input default=550 units=kg/m^3
% variable: ChainEngEff double input default=1
% variable: Tooth_Width double input default=1.09 units=mm
% variable: Lift_Force double input default=17 units=lbs
% variable: BatteryVoltage double input default=36 units=V
% variable: BatteryEfficiency double input default=1
% variable: ChiselEff double input default=1
% variable: Chain_Speed double output
% variable: Feed_Speed double output
% variable: Depth_of_Cut double output
% variable: Cutting_Force double output
% variable: Chain_Force double output
% variable: Req_Motor_Torque double output
% variable: Cutting_Efficiency double output
% variable: time_of_cut double output
% variable: Number_of_Cuts double output

%inputs
Length; % 101.6mm
Pitch; % 9.525mm
Sprocket_Drive_Teeth; %6
Drive_Sprocket_RPM; %3000
S_tooth_spacing; %8
BatteryCapacity; %4 default Ah
Feed_Force;
MC_Wood; %moisture percentage
Bar_Length;
Grip_Length;
MotorTorque; %0.6 N-m, this is an SDV
Density_Wood; %kg/m^3 CDF, default 550
ChainEngEff; %CDF from task
Tooth_Width; %1.09 mm, now a CDF
BatteryVoltage; %V, this is a CDF
Lift_Force; %17lbs, CDF now

BatteryCapacity=BatteryCapacity*BatteryEfficiency; %tie-in for uncertainty
Drive_Sprocket_w = Drive_Sprocket_RPM*0.10472; %rad/s
Area = Length*Tooth_Width; %mm^2
Engagement_Length = Bar_Length/2; %m
EnergyAvailable = BatteryVoltage*BatteryCapacity*60*60; %J
Coefficients = [109.56,-2.39,0.16,-0.62,264.83,62.77,-8.30,0.31;88.24,-2.20,0.15,-0.65,238.61,41.02,-6.68,0.30;43.03,-1.75,0.15,-0.33,131.68,98.63,-6.59,0.37];
Gearbox_Ratio = 3.5;
Gearbox_Efficiency = 0.95; %

%set values all derived from data, have to stay same for equations to work
MC_Ave = 20.57;
Density_Ave = 544.8;
Chain_Speed_Ave = 6.51; %m/s
Depth_Ave = 0.362; %mm
Depth_Overload = 0.5; %mm

if Bar_Length <0.2
    Bar_Length = 0.2;
elseif Bar_Length >0.5
    Bar_Length = 0.5;
end
if Grip_Length<0.1
    Grip_Length = 0.1;
elseif Grip_Length>0.2
    Grip_Length = 0.2;
end
if Lift_Force <8
    Lift_Force = 8;
elseif Lift_Force >26
    Lift_Force = 26;
end
if MC_Wood <20
    MC_Wood = 20;
elseif MC_Wood >30
    MC_Wood = 30;
end
if Density_Wood<300
    Density_Wood = 300;
elseif Density_Wood>850
    Density_Wood = 850;
end
%ending of bounding

Feed_Force = ((Lift_Force*4.4482216153)*Grip_Length)/Engagement_Length; %N, this takes care of the lbs to kg conversion for lift force

%initialize x for feed speed calculation
syms x

%Calculated values
Chain_Speed = ((Pitch/1000)*Sprocket_Drive_Teeth*Drive_Sprocket_w)/pi; %fixed wrong equation

%Calculate feed speed based on feed force and other input values
%removed rounding
Feed_Speed_Eqn =  Feed_Force == Coefficients(3,1)...
+(Coefficients(3,2)*(MC_Wood-MC_Ave))...
+(Coefficients(3,3)*(Density_Wood-Density_Ave))...
+(Coefficients(3,4)*(Chain_Speed-Chain_Speed_Ave))...
+(Coefficients(3,5)*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Ave))...
+(Coefficients(3,6)*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Overload))...
+(Coefficients(3,7)*((MC_Wood-MC_Ave)*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Ave)))...
+(Coefficients(3,8)*((Density_Wood-Density_Ave)*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Ave)));

Feed_Speed = solve(Feed_Speed_Eqn,x);
Depth_of_Cut = (Feed_Speed/Chain_Speed)*Pitch*S_tooth_spacing;
Depth_of_Cut = Depth_of_Cut*ChainEngEff; %updates depth of cut with modifier

%if the solve above results in the depth of cut condition where b5 should 
%be 0, then it needs to be recalculated again with that set to 0
if Depth_of_Cut-Depth_Ave > Depth_Overload-Depth_Ave 
    %need to calculate chain force and cutting force now using the depth of
    %cut value from above
    
    Chain_Force = Coefficients(1,1)...
    +(Coefficients(1,2)*(MC_Wood-MC_Ave))...
    +(Coefficients(1,3)*(Density_Wood-Density_Ave))...
    +(Coefficients(1,4)*(Chain_Speed-Chain_Speed_Ave))...
    +(Coefficients(1,5)*(Depth_of_Cut-Depth_Ave))...
    +(Coefficients(1,6)*(Depth_of_Cut-Depth_Overload))...
    +(Coefficients(1,7)*((MC_Wood-MC_Ave)*(Depth_of_Cut-Depth_Ave)))...
    +(Coefficients(1,8)*((Density_Wood-Density_Ave)*(Depth_of_Cut-Depth_Ave)));

    Cutting_Force = Coefficients(2,1)...
    +(Coefficients(2,2)*(MC_Wood-MC_Ave))...
    +(Coefficients(2,3)*(Density_Wood-Density_Ave))...
    +(Coefficients(2,4)*(Chain_Speed-Chain_Speed_Ave))...
    +(Coefficients(2,5)*(Depth_of_Cut-Depth_Ave))...
    +(Coefficients(2,6)*(Depth_of_Cut-Depth_Overload))...
    +(Coefficients(2,7)*((MC_Wood-MC_Ave)*(Depth_of_Cut-Depth_Ave)))...
    +(Coefficients(2,8)*((Density_Wood-Density_Ave)*(Depth_of_Cut-Depth_Ave)));
else %fails condition so b5 set to 0
    Feed_Speed_Eqn =  Feed_Force == Coefficients(3,1)...
    +(Coefficients(3,2)*(MC_Wood-MC_Ave))...
    +(Coefficients(3,3)*(Density_Wood-Density_Ave))...
    +(Coefficients(3,4)*(Chain_Speed-Chain_Speed_Ave))...
    +(Coefficients(3,5)*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Ave))...
    +(Coefficients(3,6)*0*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Overload))...
    +(Coefficients(3,7)*((MC_Wood-MC_Ave)*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Ave)))...
    +(Coefficients(3,8)*((Density_Wood-Density_Ave)*(((x/Chain_Speed)*Pitch*S_tooth_spacing)-Depth_Ave)));
    Feed_Speed = solve(Feed_Speed_Eqn,x);
    Depth_of_Cut = (Feed_Speed/Chain_Speed)*Pitch*S_tooth_spacing;
    %need to calculate chain force and cutting force now using the depth of
    %cut value from above, below is implementing the random values again
    %since the depth of cut is solved for again
    %also removed rounding
    Depth_of_Cut = Depth_of_Cut*ChainEngEff;

    Chain_Force = Coefficients(1,1)...
    +(Coefficients(1,2)*(MC_Wood-MC_Ave))...
    +(Coefficients(1,3)*(Density_Wood-Density_Ave))...
    +(Coefficients(1,4)*(Chain_Speed-Chain_Speed_Ave))...
    +(Coefficients(1,5)*(Depth_of_Cut-Depth_Ave))...
    +(Coefficients(1,6)*0*(Depth_of_Cut-Depth_Overload))...
    +(Coefficients(1,7)*((MC_Wood-MC_Ave)*(Depth_of_Cut-Depth_Ave)))...
    +(Coefficients(1,8)*((Density_Wood-Density_Ave)*(Depth_of_Cut-Depth_Ave)));

    Cutting_Force = Coefficients(2,1)...
    +(Coefficients(2,2)*(MC_Wood-MC_Ave))...
    +(Coefficients(2,3)*(Density_Wood-Density_Ave))...
    +(Coefficients(2,4)*(Chain_Speed-Chain_Speed_Ave))...
    +(Coefficients(2,5)*(Depth_of_Cut-Depth_Ave))...
    +(Coefficients(2,6)*0*(Depth_of_Cut-Depth_Overload))...
    +(Coefficients(2,7)*((MC_Wood-MC_Ave)*(Depth_of_Cut-Depth_Ave)))...
    +(Coefficients(2,8)*((Density_Wood-Density_Ave)*(Depth_of_Cut-Depth_Ave)));
end

%convert sym values to doubles to read easier, no rounding
Feed_Speed = double(Feed_Speed);
Depth_of_Cut = double(Depth_of_Cut);
Chain_Force = Chain_Force*ChiselEff; %chisel efficiency
Chain_Force = double(Chain_Force);
Cutting_Force = double(Cutting_Force);

%With the chain force the required motor torque can be calculated
Req_Sprocket_Torque = ((Pitch/1000)*Sprocket_Drive_Teeth*Chain_Force)/pi; %added sprocket torque vs motor torque
Req_Motor_Torque = (Req_Sprocket_Torque/Gearbox_Ratio)*Gearbox_Efficiency; %motor torque is just the iteration sprocket torque divided by gear ratio

%cutting efficiency can be calculated as well - increases for larger wood
%pieces, a square wood piece may be easiest to calculate
Cutting_Efficiency = ((Feed_Speed*1000)*Length)/((Chain_Force/Gearbox_Ratio)*Chain_Speed); %mm^2/j

%calculating against performance attributes
time_of_cut = (Length/1000)/Feed_Speed; %remained the same
%modified cutting energy required below by changing the area calculation - this became the width of the tooth multiplied by the length of cut
%previously area was the face area (length * length)

MotorW = Drive_Sprocket_w*Gearbox_Ratio;
WorkMotor = Req_Motor_Torque*MotorW*time_of_cut;
MotorEfficiencyRatioTorque = Req_Motor_Torque/MotorTorque; %new motor eff, no longer tri distr
MotorEfficiencyTorque = (-1.521*MotorEfficiencyRatioTorque^2)+(1.9801*MotorEfficiencyRatioTorque)+0.1309;
MotorEfficiencyRPM = (-7.08722426400E-11*Drive_Sprocket_RPM^3)+(3.67910058414E-07*Drive_Sprocket_RPM^2)+(-3.44053163761E-04*Drive_Sprocket_RPM)+(4.89423113119E-01);
MotorEnergyRequired = WorkMotor/(MotorEfficiencyTorque*MotorEfficiencyRPM);
Number_of_Cuts = EnergyAvailable/MotorEnergyRequired; %this is the new cut num