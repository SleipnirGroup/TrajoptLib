// Derived from WPILib code:
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sleipnirgroup.util;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;

public final class DependencyExtractor<T> {
    private static String defaultExtractionRoot;

    /**
     * Gets the default extration root location (~/.trajoptlib/nativecache).
     *
     * @return The default extraction root location.
     */
    public static synchronized String getDefaultExtractionRoot() {
        if (defaultExtractionRoot != null) {
            return defaultExtractionRoot;
        }
        String home = System.getProperty("user.home");
        defaultExtractionRoot = Path.of(home, ".trajoptlib", "nativecache").toString();
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
    public DependencyExtractor(String libraryName, String extractionRoot, Class<T> cls) {
        this.libraryName = libraryName;
        this.extractionRoot = extractionRoot;
        this.loadClass = cls;
    }

    /**
     * Loads a native library.
     *
     * @throws IOException if the library fails to load
     */
    @SuppressWarnings("PMD.PreserveStackTrace")
    public void loadLibrary() throws IOException {
        String resname = RuntimeDetector.getLibraryFileResource(libraryName);
        File dependencyLib = new File(extractionRoot, resname);
        if (!dependencyLib.exists()) {
            // If it hasn't already been extracted:
            try (InputStream resIs = loadClass.getResourceAsStream(resname)) {
                if (resIs == null) {
                    throw new IOException("A dependency is missing in jar, please contact the developer.");
                }

                var parentFile = dependencyLib.getParentFile();
                if (parentFile == null) {
                    throw new IOException("JNI library has no parent file");
                }
                parentFile.mkdirs();

                try (OutputStream os = Files.newOutputStream(dependencyLib.toPath())) {
                    byte[] buffer = new byte[0xFFFF]; // 64K copy buffer
                    int readBytes;
                    while ((readBytes = resIs.read(buffer)) != -1) { // NOPMD
                        os.write(buffer, 0, readBytes);
                    }
                }
            }
        }
    }
}
