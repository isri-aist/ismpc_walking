{
  description = "ismpc-walking-controller";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";
    gepetto.follows = "mc-rtc-nix/gepetto";
    private-trigger.follows = "mc-rtc-nix/private-trigger";
    ccache-trigger.follows = "mc-rtc-nix/ccache-trigger";
  };

  outputs =
    inputs:
    inputs.mc-rtc-nix.lib.mkMcRtcController inputs "ismpc-walking-controller" (
      { lib, ... }:
      {
        mc-rtc-nix.overlays.private = inputs.private-trigger.value;
        mc-rtc-nix.overlays.ccache = inputs.ccache-trigger.value;
        flakoboros = {
          overrideAttrs.ismpc-walking-controller = {
            src = lib.cleanSource ./.;
          };
        };
      }
    );
}
