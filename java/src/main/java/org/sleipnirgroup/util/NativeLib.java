package org.sleipnirgroup.util;

import java.io.InputStream;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.io.IOException;

public class NativeLib {
  public String hash;
  public String name;

  public NativeLib() {
  }

  @Override
  public boolean equals(Object other) {
    if (other == null) {
      return false;
    }
    if (!(other instanceof NativeLib)) {
      return false;
    }
    NativeLib otherNativeLib = (NativeLib) other;
    return this.hash.equals(otherNativeLib.hash) &&
           this.name.equals(otherNativeLib.name);
  }

  public void extractToCache(Path cachePlatformDir) throws IOException {
    String jarLibResourceName = RuntimeDetector.getLibraryFileResource(name);
    Path cachedLibPath = cachePlatformDir.resolve(name);
    try (InputStream jarLibStream = PluginLoader.class.getResourceAsStream(jarLibResourceName)) {
      if (jarLibStream == null) {
        throw new IOException("Failed to load embedded resource \"" + jarLibResourceName + "\"");
      }

      Files.createDirectories(cachePlatformDir);

      try (OutputStream os = Files.newOutputStream(cachedLibPath)) {
        byte[] buffer = new byte[0xFFFF]; // 64K copy buffer
        int readBytes;
        while ((readBytes = jarLibStream.read(buffer)) != -1) { // NOPMD
          os.write(buffer, 0, readBytes);
        }
        System.out.println("Cached new library: " + cachedLibPath + " with md5 hash of " + hash);
      }
    }
  }

  /**
   * Returns a load error message given the information in the provided UnsatisfiedLinkError.
   *
   * @param ule UnsatisfiedLinkError object.
   * @return A load error message.
   */
  private String getLoadErrorMessage(UnsatisfiedLinkError ule) {
    StringBuilder msg = new StringBuilder(512);
    msg.append(name)
        .append(
            " could not be loaded from path or an embedded resource.\n"
                + "\tattempted to load for platform ")
        .append(RuntimeDetector.getPlatformPath())
        .append("\nLast Load Error: \n")
        .append(ule.getMessage())
        .append('\n');
    if (RuntimeDetector.isWindows()) {
    msg.append(
        "A common cause of this error is missing the C++ runtime.\n"
            + "Download the latest at https://support.microsoft.com/en-us/help/2977003/the-latest-supported-visual-c-downloads\n");
    }
    return msg.toString();
  }
}
