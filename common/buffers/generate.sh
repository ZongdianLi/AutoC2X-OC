#!/bin/sh
mkdir -p build
cd build
rm *.pb.cc *.pb.h
cd ..

protoc --proto_path=./ --cpp_out=build/ cam.proto 

protoc --proto_path=./ --cpp_out=build/ data.proto 

protoc --proto_path=./ --cpp_out=build/ gps.proto 

protoc --proto_path=./ --cpp_out=build/ obd2.proto 

protoc --proto_path=./ --cpp_out=build/ trigger.proto 

protoc --proto_path=./ --cpp_out=build/ dccInfo.proto 

protoc --proto_path=./ --cpp_out=build/ camInfo.proto 

protoc --proto_path=./ --cpp_out=build/ ldmData.proto 

protoc --proto_path=./ --cpp_out=build/ BasicContainer.proto

protoc --proto_path=./ --cpp_out=build/ BasicVehicleHighFreqContainer.proto

protoc --proto_path=./ --cpp_out=build/ BasicVehicleLowFreqContainer.proto

protoc --proto_path=./ --cpp_out=build/ CamParameters.proto

protoc --proto_path=./ --cpp_out=build/ CoopAwareness.proto

protoc --proto_path=./ --cpp_out=build/ HighFreqContainer.proto

protoc --proto_path=./ --cpp_out=build/ ItsPduHeader.proto

protoc --proto_path=./ --cpp_out=build/ LowFreqContainer.proto

protoc --proto_path=./ --cpp_out=build/ PathPoint.proto

protoc --proto_path=./ --cpp_out=build/ TrajectoryPoint.proto

protoc --proto_path=./ --cpp_out=build/ TrajectoryWithStationId.proto

protoc --proto_path=./ --cpp_out=build/ ProtectedCommunicationZone.proto

protoc --proto_path=./ --cpp_out=build/ RsuHighFreqContainer.proto

protoc --proto_path=./ --cpp_out=build/ SpecialVehicleContainer.proto

protoc --proto_path=./ --cpp_out=build/ DENMessage.proto

protoc --proto_path=./ --cpp_out=build/ DENMManagementContainer.proto

protoc --proto_path=./ --cpp_out=build/ denm.proto

protoc --proto_path=./ --cpp_out=build/ mcm.proto

protoc --proto_path=./ --cpp_out=build/ ManeuverCoordination.proto

protoc --proto_path=./ --cpp_out=build/ ManeuverContainer.proto

protoc --proto_path=./ --cpp_out=build/ McmPrameters.proto

protoc --proto_path=./ --cpp_out=build/ IntentionRequestContainer.proto

protoc --proto_path=./ --cpp_out=build/ IntentionReplyContainer.proto

protoc --proto_path=./ --cpp_out=build/ PrescriptionContainer.proto

protoc --proto_path=./ --cpp_out=build/ AcceptanceContainer.proto

protoc --proto_path=./ --cpp_out=build/ HeartbeatContainer.proto

protoc --proto_path=./ --cpp_out=build/ AckContainer.proto

protoc --proto_path=./ --cpp_out=build/ FinContainer.proto
