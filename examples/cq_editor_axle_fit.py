from modular_arm.assembly.axle_fit import fit_parts
for name, part in fit_parts().items():
    show_object(part, name=name)
