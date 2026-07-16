{
  description = "ismpc-walking-controller";

  #inputs.mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
  inputs.mc-rtc-nix.url = "path:/home/arnaud/devel/mc-rtc-nix/nixpkgs";

  outputs =
    inputs:
    inputs.mc-rtc-nix.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overrideAttrs.ismpc-walking-controller = {
          src = lib.cleanSource ./.;
        };
      }
    );
}
