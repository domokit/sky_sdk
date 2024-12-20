# Updating mobile provisioning profile CIPD package

Once per year, the iOS development signing certificate used by devicelab and
chrome bots expires and a new one must be issued. Once the new certificate has
been created, a new provisioning profile needs to be created that will allow
apps signed with both the old and new certificate to run on physical test
devices.

These instructions cover one sub-step of the process of renewing our DeviceLab
development signing certificates. For the full set of instructions, which are
Google-internal, see
[How to renew the DeviceLab development certificate][renew_dev_cert].

[renew_dev_cert]: https://g3doc.corp.google.com/company/teams/flutter/infrastructure/devicelab/apple_cert_renewal.md


## Steps

1. Request write access via http://go/flutter-luci-cipd#requesting-write-read-access-to-cipd-packages.

2. Wait about 5 minutes for access rights to sync.

3. Copy the updated iOS provisioning profile to a file named `development.mobileprovision` in this directory.

4. Run `cipd create --pkg-def mac-arm64.yaml`.

5. Verify the package has been uploaded at: https://chrome-infra-packages.appspot.com/p/flutter_internal/mac/mobileprovision/mac-arm64

6. Click on the latest upload and copy the `Instance_ID` value.

7. Run `cipd create --pkg-def mac-amd64.yaml`.

8. Verify the package has been uploaded at: https://chrome-infra-packages.appspot.com/p/flutter_internal/mac/mobileprovision/mac-amd64

9. Click on the latest upload and copy the `Instance_ID` value.

10. Set the `latest` ref to the latest arm64 upload via the following command, replacing with the instance ID copied above:

   ```sh
   cipd set-ref flutter_internal/mac/mobileprovision/mac-arm64 -ref latest -version ARM64_INSTANCE_ID
   ```

11. Set the `latest` ref to the latest amd64 upload via the following command, replacing with the instance ID copied above:

   ```sh
   cipd set-ref flutter_internal/mac/mobileprovision/mac-amd64 -ref latest -version AMD64_INSTANCE_ID
   ```

12. Set the `version:to_2025` (or appropriate year) tag on the latest arm64 upload via the following command:

   ```sh
   cipd set-tag flutter_internal/mac/mobileprovision/mac-arm64 -tag "version:to_2025" -version ARM64_INSTANCE_ID
   ```

13. Set the `version:to_2025` (or appropriate year) tag on the latest amd64 upload via the following command:

   ```sh
   cipd set-tag flutter_internal/mac/mobileprovision/mac-amd64 -tag "version:to_2025" -version AMD64_INSTANCE_ID
   ```

14. Update `.ci.yaml` and migrate `apple_signing` steps to the new version tag.  
    Before: `{"dependency": "apple_signing", "version": "version:to_2024"}`  
    After: `{"dependency": "apple_signing", "version": "version:to_2025"}`
