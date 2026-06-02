import Camera_Calibration_Jakob.intrinsic_calibration_core as core


def main():
    args = core.parse_args()
    if args.list_devices:
        core.print_devices(args)
        return
    core.run_camera("insta360_front", args)


if __name__ == "__main__":
    main()
