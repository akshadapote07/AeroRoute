#include "Co4.h"

#include <array>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "AssemblyEngine.h"
#include "DroneRegisters.h"

using namespace std;

extern "C" void aeroroute_pipeline_schedule_asm(unsigned char* stageCodes);

namespace {

// Each row maps an instruction to the signals the educational control unit
// would select; this table is a model, not a hardware control implementation.
void displayControlUnitDemo() {
    const array<const char*, 8> headings = {
        "Instruction", "ALUSrc", "ALUOp", "RegWrite",
        "MemRead", "MemWrite", "Branch", "FlagWrite"
    };
    // Columns match the headings: ALUSrc, ALUOp, register/memory, branch, flags.
    const array<array<const char*, 8>, 8> signals = {{
        {{"MOV", "IMM", "PASS", "1", "0", "0", "0", "0"}},
        {{"ADD", "REG/IMM", "ADD", "1", "0", "0", "0", "1"}},
        {{"SUB", "REG", "SUB", "1", "0", "0", "0", "1"}},
        {{"CMP", "REG", "SUB", "0", "0", "0", "0", "1"}},
        {{"JG/JCC", "FLAGS", "PASS", "0", "0", "0", "1", "0"}},
        {{"SHL", "REG", "SHIFT", "1", "0", "0", "0", "0"}},
        {{"SCASB", "MEM", "CMP", "0", "1", "0", "0", "1"}},
        {{"MOVSB", "MEM", "COPY", "0", "1", "1", "0", "0"}}
    }};

    cout << "\n  --- Control Unit: illustrative control-signal table ---\n";
    for (const char* heading : headings) cout << left << setw(13) << heading;
    cout << "\n";
    for (const auto& row : signals) {
        for (const char* value : row) cout << left << setw(13) << value;
        cout << "\n";
    }
    cout << "  The Control Unit decodes opcodes into ALU, register, memory, "
            "flag, and branch signals.\n"
         << "  Hardwired control uses decode logic (fast, less flexible); "
            "microprogrammed control reads control words from memory "
            "(easier to change, generally slower).\n";
}

// Five-stage pipeline demonstration starts here.
// Independent coordinate loads are staggered by one cycle to illustrate
// overlap through IF, ID, EX, MEM, and WB without data-hazard stalls.
// Co3Asm.S generates the stage-code grid; this function prints its labels.
void displayFiveStagePipelineDemo() {
    const array<const char*, 4> instructions = {
        "MOV R1, fromCol", "MOV R2, toCol",
        "MOV R3, fromRow", "MOV R4, toRow"
    };
    const array<const char*, 5> stages = {"IF", "ID", "EX", "MEM", "WB"};
    array<array<unsigned char, 8>, 4> stageCodes{};
    aeroroute_pipeline_schedule_asm(stageCodes[0].data());

    cout << "\n  --- Five-stage pipeline timing (independent MOV instructions) ---\n";
    cout << left << setw(22) << "Instruction";
    for (int cycle = 1; cycle <= 8; ++cycle) {
        cout << setw(7) << ("C" + to_string(cycle));
    }
    cout << "\n";

    for (size_t instruction = 0; instruction < instructions.size();
         ++instruction) {
        cout << left << setw(22) << instructions[instruction];
        for (size_t cycle = 0; cycle < 8; ++cycle) {
            const unsigned char stageCode = stageCodes[instruction][cycle];
            if (stageCode > 0 && stageCode <= stages.size()) {
                cout << setw(7) << stages[stageCode - 1];
            } else {
                cout << setw(7) << "";
            }
        }
        cout << "\n";
    }
    cout << "  IF=fetch, ID=decode, EX=execute, MEM=memory, WB=write-back; "
            "this is a processor-model demonstration.\n";
}
// Five-stage pipeline demonstration ends here.

// This presents the stages for the already-checked ADD result; it is a
// processor-level explanation, not a hardware pipeline execution trace.
void displayAddInstructionFlow(
    long long address, long long startingAltitude,
    long long climb, long long result) {
    cout << "\n  --- Fetch/decode/execute trace: ADD RAX, " << climb << " ---\n"
         << "  IF:  fetch at " << hex << uppercase << "0x" << address << dec
         << "\n"
         << "  ID:  decode ADD; read RAX=" << startingAltitude
         << " and immediate " << climb
         << "; select ALU=ADD, RegWrite=1\n"
         << "  EX:  ALU computes " << startingAltitude << " + "
         << climb << " = " << result << "\n"
         << "  MEM: no data-memory access\n"
         << "  WB:  write " << result << " to RAX\n";
}

} // namespace

void runCoaCo4() {
    cout << "\n--- COA CO4: Integrated processor-level mission workflow ---\n";
    DroneRegisters registers;
    AssemblyEngine assembly(registers);
    assembly.clearLog();

    // Map application telemetry into the CPU-register model.
    registers.loadTelemetry(120, 18.5, 45.0, 87.0);
    cout << "  Application telemetry loaded into CPU registers:\n";
    registers.displayRegisters();
    // Calculate route displacement in the external x86-64 assembly routine.
    const DisplacementData displacement =
        assembly.computeDisplacement(0, 0, 7, 5);
    const long long startingAltitude = registers.getRAX();
    const long long addInstructionAddress = registers.getPC();
    // Demonstrate an ALU ADD, then write its result back to the RAX model.
    const long long altitude = assembly.SIM_ADD(
        startingAltitude, 15, "RAX", "15");
    registers.MOV_RAX(altitude);
    // Compare battery telemetry against 20% and model the conditional branch.
    const bool dispatchAllowed = assembly.SIM_CMP_JMP(
        registers.getRDX(), 2000, "JG", "RDX", "2000");
    // Compare the original repeated-addition example with assembly algorithms.
    const long long multipliedRouteCost =
        assembly.SIM_MUL_SUCCESSIVE(9, 6);
    const long long addShiftRouteCost =
        assembly.SIM_MUL_ADD_SHIFT(9, 6);
    const long long boothRouteCost = assembly.SIM_MUL_BOOTH(9, 6);
    // Check both positive and negative signed division behavior.
    const IntegerDivisionResult dividedRouteCost =
        assembly.SIM_DIV_SHIFT_SUB(boothRouteCost, 6);
    const IntegerDivisionResult signedDivision =
        assembly.SIM_DIV_SHIFT_SUB(-boothRouteCost, 6);
    // Exercise assembly string scanning and appending on mission text.
    string missionText = "AR-DRONE";
    const size_t originalLength = assembly.asmStringLength(missionText);
    assembly.asmStringAppend(missionText, "-READY");

    if (displacement.dx != 5.0 || displacement.dy != 7.0 ||
        altitude != 135 || !dispatchAllowed ||
        multipliedRouteCost != 54 || originalLength != 8 ||
        addShiftRouteCost != multipliedRouteCost ||
        boothRouteCost != multipliedRouteCost ||
        dividedRouteCost.quotient != 9 ||
        dividedRouteCost.remainder != 0 ||
        signedDivision.quotient != -9 ||
        signedDivision.remainder != 0 ||
        missionText != "AR-DRONE-READY") {
        throw runtime_error("COA CO4 integrated mission workflow failed.");
    }

    displayAddInstructionFlow(
        addInstructionAddress, startingAltitude, 15, altitude);
    displayControlUnitDemo();
    displayFiveStagePipelineDemo();

    cout << "  Application coordinates -> displacement: ("
         << displacement.dx << ", " << displacement.dy
         << "), distance=" << displacement.total << "\n"
         << "  ALU/register update: altitude RAX=" << registers.getRAX()
         << " m; conditional branch: "
         << (dispatchAllowed ? "dispatch allowed" : "hold for charging")
         << "\n"
         << "  Arithmetic result: 9 x 6 = " << multipliedRouteCost << "\n"
         << "  ADD-and-shift / Booth multiplication: "
         << addShiftRouteCost << " / " << boothRouteCost << "\n"
         << "  Division result: " << multipliedRouteCost
         << " / 6 = " << dividedRouteCost.quotient
         << " remainder " << dividedRouteCost.remainder << "\n"
         << "  64-bit string processing: " << missionText << "\n";
    assembly.displayInstructionLog();
    cout << "COA CO4 checks passed: application logic is mapped to registers, "
            "ALU instructions, control flow, assembly and string processing.\n";
    registers.exportToJson(assembly.getInstructionLog(), displacement);
}
