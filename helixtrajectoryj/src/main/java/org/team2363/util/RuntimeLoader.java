// Derived from WPILib code:
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2363.util;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Scanner;

public final class RuntimeLoader<T> {
  
    private static String defaultExtractionRoot;

    /**
     * Gets the default extration root location (~/.wpilib/nativecache).
     *
     * @return The default extraction root location.
     */
    public static synchronized String getDefaultExtractionRoot() {
        if (defaultExtractionRoot != null) {
            return defaultExtractionRoot;
        }
        String home = System.getProperty("user.home");
        defaultExtractionRoot = Paths.get(home, ".helixtrajectory", "nativecache").toString();
        return defaultExtractionRoot;
    }

    private final String libraryName;
    private final String extractionRoot;
    private final Class<T> loadClass;

    /**
     * Creates a new library loader.
     *
     * @param libraryName Name of library to load.
     * @param extractionRoot Location from which to load the library.
     * @param cls Class whose classpath the given library belongs.
     */
    public RuntimeLoader(String libraryName, String extractionRoot, Class<T> cls) {
        this.libraryName = libraryName;
        this.extractionRoot = extractionRoot;
        this.loadClass = cls;
    }

    /**
     * Returns a load error message given the information in the provided UnsatisfiedLinkError.
     *
     * @param ule UnsatisfiedLinkError object.
     * @return A load error message.
     */
    private String getLoadErrorMessage(UnsatisfiedLinkError ule) {
        StringBuilder msg = new StringBuilder(512);
        msg.append(libraryName)
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

    /**
     * Loads a native library.
     *
     * @throws IOException if the library fails to load
     */
    @SuppressWarnings("PMD.PreserveStackTrace")
    public void loadLibrary() throws IOException {
        try {
            // First, try loading path
            System.loadLibrary(libraryName);
        } catch (UnsatisfiedLinkError ule) {
            // Then load the hash from the resources
            String hashName = RuntimeDetector.getHashLibraryResource(libraryName);
            String resname = RuntimeDetector.getLibraryResource(libraryName);
            try (InputStream hashIs = loadClass.getResourceAsStream(hashName)) {
                if (hashIs == null) {
                    throw new IOException(getLoadErrorMessage(ule));
                }
                try (Scanner scanner = new Scanner(hashIs, StandardCharsets.UTF_8.name())) {
                    String hash = scanner.nextLine();
                    File jniLibrary = new File(extractionRoot, resname + "." + hash);
                    try {
                        // Try to load from an already extracted hash
                        System.load(jniLibrary.getAbsolutePath());
                    } catch (UnsatisfiedLinkError ule2) {
                        // If extraction failed, extract
                        try (InputStream resIs = loadClass.getResourceAsStream(resname + "." + hash)) {
                            if (resIs == null) {
                                throw new IOException(getLoadErrorMessage(ule));
                            }

                            var parentFile = jniLibrary.getParentFile();
                            if (parentFile == null) {
                                throw new IOException("JNI library has no parent file");
                            }
                            parentFile.mkdirs();

                            try (OutputStream os = Files.newOutputStream(jniLibrary.toPath())) {
                                byte[] buffer = new byte[0xFFFF]; // 64K copy buffer
                                int readBytes;
                                while ((readBytes = resIs.read(buffer)) != -1) { // NOPMD
                                    os.write(buffer, 0, readBytes);
                                }
                            }
                            System.load(jniLibrary.getAbsolutePath());
                        }
                    }
                }
            }
        }
    }
}