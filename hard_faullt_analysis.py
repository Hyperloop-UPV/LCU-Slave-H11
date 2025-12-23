import subprocess
import struct
import re
import os
HF_FLASH_ADDR = 0x080C0000
HF_FLASH_ADDR_STRING = "0x080C000"
ELF_FILE = "out/build/latest.elf"

def read_flash():
    cmd = [
        "STM32_Programmer_CLI",
        "-c", "port=SWD",
        "-r32", hex(HF_FLASH_ADDR), "44"
    ]
    out = subprocess.check_output(cmd, text=True)
    return out

def decode_cfsr_memory(cfsr, fault_addr):
    memory_fault = cfsr & 0xFF
    if memory_fault == 0:
        return
    print("\nMemory Fault (MMFSR):")
    if memory_fault & 0b10000000:
        print(f"  MMARVALID: Memory fault address valid -> 0x{fault_addr:08X}")
    if memory_fault & 0b00100000:
        print("  MLSPERR : Floating Point Unit lazy state preservation error")
    if memory_fault & 0b00010000:
        print("  MSTKERR : Stack error on entry to exception")
    if memory_fault & 0b00001000:
        print("  MUNSTKERR : Stack error on return from exception")
    if memory_fault & 0b00000010:
        print("  DACCVIOL : Data access violation (NULL pointer or invalid access)")
    if memory_fault & 0b00000001:
        print("  IACCVIOL : Instruction access violation")


# --------------------------
# Decode Bus Fault (BFSR)
# --------------------------
def decode_cfsr_bus(cfsr, fault_addr):
    bus_fault = (cfsr & 0x0000FF00) >> 8
    if bus_fault == 0:
        return
    print("\nBus Fault (BFSR):")
    if bus_fault & 0b10000000:
        if(bus_fault & 0b00000001):
            print(f"  BFARVALID : Bus fault address valid -> 0x{fault_addr:08X}")
        else: 
            print(f"  BFARVALID : Bus fault address imprecise -> 0x{fault_addr:08X}")
    if bus_fault & 0b00100000:
        print("  LSPERR : Floating Point Unit lazy state preservation error")
    if bus_fault & 0b00010000:
        print("  STKERR : Stack error on entry to exception")
    if bus_fault & 0b00001000:
        print("  UNSTKERR : Stack error on return from exception")


# --------------------------
# Decode Usage Fault (UFSR)
# --------------------------
def decode_cfsr_usage(cfsr):
    usage_fault = (cfsr & 0xFFFF0000) >> 16
    if usage_fault == 0:
        return
    print("\nUsage Fault (UFSR):")
    if usage_fault & 0x0200:
        print("  DIVBYZERO : Division by zero")
    if usage_fault & 0x0100:
        print("  UNALIGNED : Unaligned memory access")
    if usage_fault & 0x0008:
        print("  NOCP : Accessed FPU when not present")
    if usage_fault & 0x0004:
        print("  INVPC : Invalid Program Counter(PC) load")
    if usage_fault & 0x0002:
        print("  INVSTATE : Invalid processor state")
    if usage_fault & 0x0001:
        print("  UNDEFINSTR : Undefined instruction")


def decode_cfsr(cfsr, fault_addr):
    decode_cfsr_memory(cfsr, fault_addr)
    decode_cfsr_bus(cfsr, fault_addr)
    decode_cfsr_usage(cfsr)
    
        
def addr2line(addr):
    cmd = ["arm-none-eabi-addr2line", "-e", ELF_FILE, "-f", "-C", hex(addr)]
    try:
        output = subprocess.check_output(cmd, text=True).strip()
        return output
    except Exception as e:
        return f"addr2line failed: {e}"

def print_code_context(lines, context=2):
    """
    lines: salida de addr2line (función + file:line)
    context: cuántas líneas arriba/abajo mostrar
    """
    line_list = lines.splitlines()
    if len(line_list) < 2:
        print("Invalid addr2line output")
        return

    file_line = line_list[1].strip()  # ejemplo: /path/to/file.cpp:5
    split = file_line.rfind(':')
    file_path = file_line[:split]
    line_no = int(file_line[split+1:]) - 1  # índice base 0

    if not os.path.exists(file_path):
        print("Source file not found")
        return

    with open(file_path, "r") as f:
        file_lines = f.readlines()

    start = max(0, line_no - context)
    end = min(len(file_lines), line_no + context + 1)

    print(f"\nSource snippet from {file_path}:")
    for i in range(start, end):
        code = file_lines[i].rstrip()
        # Si es la línea del error, la ponemos en rojo
        if i == line_no:
            print(f"\033[91m{i+1:>4}: {code}\033[0m")  # rojo
        else:
            print(f"{i+1:>4}: {code}")

def hard_fault_analysis(memory_string):
    raw = bytes.fromhex(memory_string)
    raw = struct.unpack(">11I",raw)
    hf = {
        "HF_Flag": raw[0],
        "r0": raw[1],
        "r1": raw[2],
        "r2": raw[3],
        "r3": raw[4],
        "r12": raw[5],
        "lr": raw[6],
        "pc":raw[7],
        "psr": raw[8],
        "cfsr":raw[9],
        "fault_addr":raw[10]
    }
    if(hf["HF_Flag"] != 0xFF00FF00):
        print("There was no hardfault in your Microcontroller, Kudos for you, I hope...")
        return
    print("================HARDFAULT DETECTED ===========")
    print("Registers:")
    
    for r in ['r0','r1','r2','r3','r12','lr','pc','psr']:
        print(f"  {r.upper():<4}: 0x{hf[r]:08X}")
    
    print(f"  CFSR: 0x{hf['cfsr']:08X}")
    decode_cfsr(hf["cfsr"], hf["fault_addr"])
    print("\nSource Location:")
    pc_loc = addr2line(hf["pc"])
    lr_loc = addr2line(hf["lr"])
    print(f"  Linker Register : 0x{hf['lr']:08X} -> {lr_loc}")
    
    print(f"  Program Counter : 0x{hf['pc']:08X} -> {pc_loc}")
    print_code_context(pc_loc)
    
    
    print("======================================================")
    
    
    print("Note: In Release builds (-O2/-O3) the PC may not point exactly to the failing instruction.")
    print("      During interrupts, bus faults, or stack corruption, the PC can be imprecise.")
    print("      Use the LR and stack frame for better context of where the fault originated.")

if __name__ == '__main__':
    out = read_flash()
    pos_memory_flash = out.rfind(HF_FLASH_ADDR_STRING)
    print(out[0:pos_memory_flash])
    flash = out[pos_memory_flash:]
    print(flash)
    memory_string = ""
    for line in flash.splitlines():
        if(line.find(':') == -1):
            break
        _,mem = line.split(":")
        memory_string += mem
    memory_string = memory_string.replace(" ","")
    print(memory_string)
    hard_fault_analysis(memory_string)
    
