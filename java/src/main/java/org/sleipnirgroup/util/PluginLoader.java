package org.sleipnirgroup.util;

import java.io.IOException;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.util.stream.Collectors;
import java.nio.file.Files;
import java.nio.file.Path;
import java.io.FileWriter;

import org.sleipnirgroup.trajopt.PluginLoadException;

import com.fasterxml.jackson.databind.ObjectMapper;

public class PluginLoader {

  private static boolean isPluginLoaded = false;

  public static void loadPlugin() throws PluginLoadException {
    if (!isPluginLoaded) {
      String platformPath = RuntimeDetector.getPlatformPath();
      var jarNativeLibsStream = PluginLoader.class.getResourceAsStream(RuntimeDetector.getLibraryFileResource("native_libs.json"));

      ObjectMapper mapper = new ObjectMapper();
      // Credit: https://stackoverflow.com/questions/309424/how-do-i-read-convert-an-inputstream-into-a-string-in-java
      String jarNativeLibsJSON = new BufferedReader(new InputStreamReader(jarNativeLibsStream)).lines().collect(Collectors.joining("\n"));

      NativeLibs jarNativeLibs;
      try {
        jarNativeLibs = mapper.readValue(jarNativeLibsJSON, NativeLibs.class);
      } catch (IOException e) {
        throw new PluginLoadException("Could not load embedded native libraries json.", e);
      }
      
      Path cacheDir = Path.of(System.getProperty("user.home"), ".trajoptlib", "nativecache");
      Path cachePlatformPath = cacheDir.resolve(platformPath);
      Path cacheNativeLibsPath = cachePlatformPath.resolve("native_libs.json");

      NativeLibs cacheNativeLibs = null;
      try {
        cacheNativeLibs = mapper.readValue(cacheNativeLibsPath.toFile(), NativeLibs.class);
      } catch (IOException e) {
        // Assume there is no cache, so don't do anything
      }

      // Delete old cached lib files
      if (cacheNativeLibs != null) {
        for (NativeLib cachedLib : cacheNativeLibs.libs) {
          if (!jarNativeLibs.libs.contains(cachedLib)) {
            Path excessCachedLib = cachePlatformPath.resolve(cachedLib.name);
            try {
              Files.delete(excessCachedLib);
            } catch (IOException e) {
              throw new PluginLoadException("Could not delete outdated cached native library at " + excessCachedLib.toAbsolutePath().toString() + "", e);
            }
          }
        }
      }

      // Add new cached libs that aren't already there
      for (NativeLib jarLib : jarNativeLibs.libs) {
        if (cacheNativeLibs == null || !cacheNativeLibs.libs.contains(jarLib)) {
          try {
            jarLib.extractToCache(cacheDir.resolve(platformPath));
          } catch (IOException e) {
            throw new PluginLoadException("Failed to extract and cache an embedded resource", e);
          }
        }
      }
      
      // Load the jni library
      try {
        Path jniLibPath = cachePlatformPath.resolve(jarNativeLibs.jni);
        System.load(jniLibPath.toAbsolutePath().toString());
      } catch (UnsatisfiedLinkError e) {
        throw new PluginLoadException("Could not load TrajoptLib plugin: " + e.getMessage(), e);
      }

      // Always write the native libs json to record what's been saved
      try {
        FileWriter fileWriter = new FileWriter(cacheNativeLibsPath.toFile());
        PrintWriter printWriter = new PrintWriter(fileWriter);
        printWriter.append(jarNativeLibsJSON);
        printWriter.close();
        fileWriter.close();
      } catch (IOException e) {
        throw new PluginLoadException("Failed to write the native libs json to cache.", e);
      }

      isPluginLoaded = true;
    }
  }
}
