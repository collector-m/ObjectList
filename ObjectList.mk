##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=ObjectList
ConfigurationName      :=Debug
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
WorkspacePath          := "/home/denggroup/Workspace/yangguorun"
ProjectPath            := "/home/denggroup/Workspace/yangguorun/ObjectList"
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=root
Date                   :=11/14/2014
CodeLitePath           :="/home/denggroup/.codelite"
LinkerName             :=g++
ArchiveTool            :=ar rcus
SharedObjectLinkerName :=g++ -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=.o.d
PreprocessSuffix       :=.o.i
DebugSwitch            :=-gstab
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
CompilerName           :=g++
C_CompilerName         :=gcc
OutputFile             :=$(HOME)/UGV/bin/ModuleObjectList
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E 
ObjectsFileList        :="/home/denggroup/Workspace/yangguorun/ObjectList/ObjectList.txt"
PCHCompileFlags        :=
MakeDirCommand         :=mkdir -p
CmpOptions             := -g -O0 -Wall $(Preprocessors)
C_CmpOptions           := -g -O0 -Wall $(Preprocessors)
LinkOptions            :=  
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch)$(HOME)/UGV/include $(IncludeSwitch). 
IncludePCH             := 
RcIncludePath          := 
Libs                   := $(LibrarySwitch)opencv_imgproc $(LibrarySwitch)opencv_core $(LibrarySwitch)opencv_highgui $(LibrarySwitch)GLU $(LibrarySwitch)glut $(LibrarySwitch)module $(LibrarySwitch)log4cxx 
LibPath                := $(LibraryPathSwitch). $(LibraryPathSwitch)$(HOME)/UGV/libs 


##
## User defined environment variables
##
CodeLiteDir:=/usr/share/codelite
Objects=$(IntermediateDirectory)/main$(ObjectSuffix) $(IntermediateDirectory)/GLFunc$(ObjectSuffix) $(IntermediateDirectory)/Object$(ObjectSuffix) $(IntermediateDirectory)/ObjectList$(ObjectSuffix) $(IntermediateDirectory)/DetectBox$(ObjectSuffix) $(IntermediateDirectory)/Point$(ObjectSuffix) $(IntermediateDirectory)/pos$(ObjectSuffix) $(IntermediateDirectory)/Line$(ObjectSuffix) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects) > $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

$(IntermediateDirectory)/.d:
	@test -d ./Debug || $(MakeDirCommand) ./Debug

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/main$(ObjectSuffix): main.cpp $(IntermediateDirectory)/main$(DependSuffix)
	$(CompilerName) $(IncludePCH) $(SourceSwitch) "/home/denggroup/Workspace/yangguorun/ObjectList/main.cpp" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/main$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main$(DependSuffix): main.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main$(ObjectSuffix) -MF$(IntermediateDirectory)/main$(DependSuffix) -MM "/home/denggroup/Workspace/yangguorun/ObjectList/main.cpp"

$(IntermediateDirectory)/main$(PreprocessSuffix): main.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main$(PreprocessSuffix) "/home/denggroup/Workspace/yangguorun/ObjectList/main.cpp"

$(IntermediateDirectory)/GLFunc$(ObjectSuffix): GLFunc.cpp $(IntermediateDirectory)/GLFunc$(DependSuffix)
	$(CompilerName) $(IncludePCH) $(SourceSwitch) "/home/denggroup/Workspace/yangguorun/ObjectList/GLFunc.cpp" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/GLFunc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/GLFunc$(DependSuffix): GLFunc.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/GLFunc$(ObjectSuffix) -MF$(IntermediateDirectory)/GLFunc$(DependSuffix) -MM "/home/denggroup/Workspace/yangguorun/ObjectList/GLFunc.cpp"

$(IntermediateDirectory)/GLFunc$(PreprocessSuffix): GLFunc.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/GLFunc$(PreprocessSuffix) "/home/denggroup/Workspace/yangguorun/ObjectList/GLFunc.cpp"

$(IntermediateDirectory)/Object$(ObjectSuffix): Object.cpp $(IntermediateDirectory)/Object$(DependSuffix)
	$(CompilerName) $(IncludePCH) $(SourceSwitch) "/home/denggroup/Workspace/yangguorun/ObjectList/Object.cpp" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/Object$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/Object$(DependSuffix): Object.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/Object$(ObjectSuffix) -MF$(IntermediateDirectory)/Object$(DependSuffix) -MM "/home/denggroup/Workspace/yangguorun/ObjectList/Object.cpp"

$(IntermediateDirectory)/Object$(PreprocessSuffix): Object.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/Object$(PreprocessSuffix) "/home/denggroup/Workspace/yangguorun/ObjectList/Object.cpp"

$(IntermediateDirectory)/ObjectList$(ObjectSuffix): ObjectList.cpp $(IntermediateDirectory)/ObjectList$(DependSuffix)
	$(CompilerName) $(IncludePCH) $(SourceSwitch) "/home/denggroup/Workspace/yangguorun/ObjectList/ObjectList.cpp" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/ObjectList$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/ObjectList$(DependSuffix): ObjectList.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/ObjectList$(ObjectSuffix) -MF$(IntermediateDirectory)/ObjectList$(DependSuffix) -MM "/home/denggroup/Workspace/yangguorun/ObjectList/ObjectList.cpp"

$(IntermediateDirectory)/ObjectList$(PreprocessSuffix): ObjectList.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/ObjectList$(PreprocessSuffix) "/home/denggroup/Workspace/yangguorun/ObjectList/ObjectList.cpp"

$(IntermediateDirectory)/DetectBox$(ObjectSuffix): DetectBox.cpp $(IntermediateDirectory)/DetectBox$(DependSuffix)
	$(CompilerName) $(IncludePCH) $(SourceSwitch) "/home/denggroup/Workspace/yangguorun/ObjectList/DetectBox.cpp" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/DetectBox$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/DetectBox$(DependSuffix): DetectBox.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/DetectBox$(ObjectSuffix) -MF$(IntermediateDirectory)/DetectBox$(DependSuffix) -MM "/home/denggroup/Workspace/yangguorun/ObjectList/DetectBox.cpp"

$(IntermediateDirectory)/DetectBox$(PreprocessSuffix): DetectBox.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/DetectBox$(PreprocessSuffix) "/home/denggroup/Workspace/yangguorun/ObjectList/DetectBox.cpp"

$(IntermediateDirectory)/Point$(ObjectSuffix): Point.cpp $(IntermediateDirectory)/Point$(DependSuffix)
	$(CompilerName) $(IncludePCH) $(SourceSwitch) "/home/denggroup/Workspace/yangguorun/ObjectList/Point.cpp" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/Point$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/Point$(DependSuffix): Point.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/Point$(ObjectSuffix) -MF$(IntermediateDirectory)/Point$(DependSuffix) -MM "/home/denggroup/Workspace/yangguorun/ObjectList/Point.cpp"

$(IntermediateDirectory)/Point$(PreprocessSuffix): Point.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/Point$(PreprocessSuffix) "/home/denggroup/Workspace/yangguorun/ObjectList/Point.cpp"

$(IntermediateDirectory)/pos$(ObjectSuffix): pos.cpp $(IntermediateDirectory)/pos$(DependSuffix)
	$(CompilerName) $(IncludePCH) $(SourceSwitch) "/home/denggroup/Workspace/yangguorun/ObjectList/pos.cpp" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/pos$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/pos$(DependSuffix): pos.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/pos$(ObjectSuffix) -MF$(IntermediateDirectory)/pos$(DependSuffix) -MM "/home/denggroup/Workspace/yangguorun/ObjectList/pos.cpp"

$(IntermediateDirectory)/pos$(PreprocessSuffix): pos.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/pos$(PreprocessSuffix) "/home/denggroup/Workspace/yangguorun/ObjectList/pos.cpp"

$(IntermediateDirectory)/Line$(ObjectSuffix): Line.cpp $(IntermediateDirectory)/Line$(DependSuffix)
	$(CompilerName) $(IncludePCH) $(SourceSwitch) "/home/denggroup/Workspace/yangguorun/ObjectList/Line.cpp" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/Line$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/Line$(DependSuffix): Line.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/Line$(ObjectSuffix) -MF$(IntermediateDirectory)/Line$(DependSuffix) -MM "/home/denggroup/Workspace/yangguorun/ObjectList/Line.cpp"

$(IntermediateDirectory)/Line$(PreprocessSuffix): Line.cpp
	@$(CompilerName) $(CmpOptions) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/Line$(PreprocessSuffix) "/home/denggroup/Workspace/yangguorun/ObjectList/Line.cpp"


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) $(IntermediateDirectory)/main$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/main$(DependSuffix)
	$(RM) $(IntermediateDirectory)/main$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/GLFunc$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/GLFunc$(DependSuffix)
	$(RM) $(IntermediateDirectory)/GLFunc$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/Object$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/Object$(DependSuffix)
	$(RM) $(IntermediateDirectory)/Object$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/ObjectList$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/ObjectList$(DependSuffix)
	$(RM) $(IntermediateDirectory)/ObjectList$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/DetectBox$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/DetectBox$(DependSuffix)
	$(RM) $(IntermediateDirectory)/DetectBox$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/Point$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/Point$(DependSuffix)
	$(RM) $(IntermediateDirectory)/Point$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/pos$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/pos$(DependSuffix)
	$(RM) $(IntermediateDirectory)/pos$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/Line$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/Line$(DependSuffix)
	$(RM) $(IntermediateDirectory)/Line$(PreprocessSuffix)
	$(RM) $(OutputFile)
	$(RM) "/home/denggroup/Workspace/yangguorun/.build-debug/ObjectList"


