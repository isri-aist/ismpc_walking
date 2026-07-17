{
  description = "ismpc-walking-controller";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";
    gepetto.follows = "mc-rtc-nix/gepetto";
    private-trigger.url = "github:boolean-option/false";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      let
        with-private = inputs.private-trigger.value;
      in
      {
        systems = [ "x86_64-linux" ];
        imports = [
          inputs.mc-rtc-nix.flakeModule
          {
            mc-rtc-nix.overlays.private = with-private;
            mc-rtc-superbuild =
              { pkgs, ... }:
              {
                enable = true;
                configurations =
                  let
                    ismpc = pkgs.ismpc-walking-controller;
                  in
                  {
                    ismpc-minimal = {
                      extends = [ "minimal" ];
                      enabled = "ismpc_walking";
                      runtime = {
                        apps = [
                          pkgs.mc-rtc-magnum
                        ];
                      };
                      devel = {
                        controllers = [ ismpc ];
                        # XXX: should be ismpc.mc-rtc.observers
                        observers = [ pkgs.mc-state-observation ];
                        plugins = ismpc.mc-rtc.plugins;
                      };
                    };
                    ismpc-full = {
                      extends = [
                        "ismpc-minimal"
                      ];
                      runtime = {
                        apps = [
                          pkgs.mc-mujoco-full
                        ];
                        robots = lib.optionals with-private [
                          pkgs.mc-hrp4
                          pkgs.mc-hrp5-p
                        ];
                      };
                    };
                  };
              };

            flakoboros = {
              overrideAttrs.ismpc-walking-controller = {
                src = lib.cleanSource ./.;
              };
            };
          }
        ];

      }
    );
}
