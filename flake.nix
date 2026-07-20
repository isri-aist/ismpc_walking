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
    inputs.mc-rtc-nix.lib.mkMcRtcController inputs "ismpc-walking-controller" (
      { lib, ... }:
      {
        mc-rtc-nix.overlays.private = inputs.private-trigger.value;
        flakoboros = {
          overrideAttrs.ismpc-walking-controller = {
            src = lib.cleanSource ./.;
          };
        };
      }
    );
}
